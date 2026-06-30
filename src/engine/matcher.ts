import type { RoadGraph, RoadEdge, MatchedSegment, PoseEstimate, GeoPoint } from '@/types/navigation';

function haversine(a: GeoPoint, b: GeoPoint): number {
  const R = 6371000;
  const dLat = (b.lat - a.lat) * Math.PI / 180;
  const dLng = (b.lng - a.lng) * Math.PI / 180;
  const sinDLat2 = Math.sin(dLat / 2);
  const sinDLng2 = Math.sin(dLng / 2);
  const c = 2 * Math.asin(Math.sqrt(sinDLat2 * sinDLat2 + Math.cos(a.lat * Math.PI / 180) * Math.cos(b.lat * Math.PI / 180) * sinDLng2 * sinDLng2));
  return R * c;
}

function pointToSegmentDistance(p: GeoPoint, a: GeoPoint, b: GeoPoint): number {
  const latDiff = b.lat - a.lat;
  const lngDiff = b.lng - a.lng;
  if (latDiff === 0 && lngDiff === 0) return haversine(p, a);
  
  const latFactor = 111320;
  const lngFactor = 111320 * Math.cos(a.lat * Math.PI / 180);
  
  const dx = lngDiff * lngFactor;
  const dy = latDiff * latFactor;
  const dpx = (p.lng - a.lng) * lngFactor;
  const dpy = (p.lat - a.lat) * latFactor;
  
  const t = Math.max(0, Math.min(1, (dpx * dx + dpy * dy) / (dx * dx + dy * dy)));
  
  const closestLng = a.lng + t * lngDiff;
  const closestLat = a.lat + t * latDiff;
  
  return haversine(p, { lat: closestLat, lng: closestLng });
}

function projectPointToSegment(p: GeoPoint, a: GeoPoint, b: GeoPoint): { point: GeoPoint; arcPos: number } {
  const latFactor = 111320;
  const lngFactor = 111320 * Math.cos(a.lat * Math.PI / 180);
  
  const dx = (b.lng - a.lng) * lngFactor;
  const dy = (b.lat - a.lat) * latFactor;
  const dpx = (p.lng - a.lng) * lngFactor;
  const dpy = (p.lat - a.lat) * latFactor;
  
  const segLen = Math.sqrt(dx * dx + dy * dy);
  const t = segLen > 0 ? Math.max(0, Math.min(1, (dpx * dx + dpy * dy) / (segLen * segLen))) : 0;
  
  const closestLng = a.lng + (t * (b.lng - a.lng));
  const closestLat = a.lat + (t * (b.lat - a.lat));
  const arcPos = t * segLen;
  
  return { point: { lat: closestLat, lng: closestLng }, arcPos };
}

interface Candidate {
  edgeId: number;
  edge: RoadEdge;
  point: GeoPoint;
  arcPos: number;
  lateralError: number;
  emissionProb: number;
  transitionProb: number;
  totalProb: number;
  prevCandidate: Candidate | null;
}

export class HMMMapMatcher {
  private prevCandidates: Candidate[] = [];
  private readonly maxCandidates = 20;
  private readonly emissionSigma = 8;
  private readonly transitionSigma = 15;

  constructor(graph: RoadGraph) { this.graph = graph; }

  match(pose: PoseEstimate): MatchedSegment | null {
    const rawPos = pose.position;
    const searchRadius = Math.max(30, pose.sigmaM * 2);

    const candidates = this.findCandidates(rawPos, searchRadius, pose.speed, pose.heading);
    if (candidates.length === 0) {
      this.prevCandidates = [];
      return null;
    }

    for (const c of candidates) {
      c.emissionProb = this.gaussian(c.lateralError, 0, this.emissionSigma);
    }

    if (this.prevCandidates.length === 0) {
      for (const c of candidates) {
        c.totalProb = c.emissionProb;
        c.prevCandidate = null;
      }
    } else {
      for (const c of candidates) {
        let bestTransProb = 0;
        let bestPrev: Candidate | null = null;

        for (const prev of this.prevCandidates) {
          const routeDist = this.routeDistance(prev.point, c.point);
          const euclideanDist = haversine(prev.point, c.point);
          const topologyScore = routeDist > 0 ? euclideanDist / routeDist : 1;

          const timeDelta = 0.1;
          const expectedDist = pose.speed * timeDelta;
          const speedError = Math.abs(routeDist - expectedDist);
          const speedScore = this.gaussian(speedError, 0, this.transitionSigma);

          const transProb = topologyScore * speedScore * prev.totalProb;
          if (transProb > bestTransProb) {
            bestTransProb = transProb;
            bestPrev = prev;
          }
        }

        c.totalProb = c.emissionProb * bestTransProb;
        c.prevCandidate = bestPrev;
      }
    }

    let best = candidates[0];
    for (const c of candidates) {
      if (c.totalProb > best.totalProb) best = c;
    }

    this.prevCandidates = candidates
      .sort((a, b) => b.totalProb - a.totalProb)
      .slice(0, this.maxCandidates);

    const confidence = Math.min(1, best.emissionProb / (candidates[0]?.emissionProb || 1));

    return {
      edgeId: best.edgeId,
      arcPos: best.arcPos,
      confidence: confidence > 0.5 ? confidence : confidence * 0.5,
      lateralError: best.lateralError,
    };
  }

  private findCandidates(pos: GeoPoint, radius: number, _speed: number, _heading: number): Candidate[] {
    const candidates: Candidate[] = [];

    for (const [edgeId, edge] of this.graph.edges) {
      const minLat = Math.min(...edge.waypoints.map(w => w.lat)) - 0.001;
      const maxLat = Math.max(...edge.waypoints.map(w => w.lat)) + 0.001;
      const minLng = Math.min(...edge.waypoints.map(w => w.lng)) - 0.001;
      const maxLng = Math.max(...edge.waypoints.map(w => w.lng)) + 0.001;

      if (pos.lat < minLat || pos.lat > maxLat || pos.lng < minLng || pos.lng > maxLng) continue;

      let minDist = Infinity;
      let bestProjection = pos;
      let bestArcPos = 0;

      for (let i = 0; i < edge.waypoints.length - 1; i++) {
        const from = edge.waypoints[i];
        const to = edge.waypoints[i + 1];
        const dist = pointToSegmentDistance(pos, from, to);
        if (dist < minDist && dist < radius) {
          minDist = dist;
          const proj = projectPointToSegment(pos, from, to);
          bestProjection = proj.point;
          bestArcPos = proj.arcPos;
          for (let j = 0; j < i; j++) {
            bestArcPos += haversine(edge.waypoints[j], edge.waypoints[j + 1]);
          }
        }
      }

      if (minDist < radius) {
        candidates.push({
          edgeId,
          edge,
          point: bestProjection,
          arcPos: bestArcPos,
          lateralError: minDist,
          emissionProb: 0,
          transitionProb: 0,
          totalProb: 0,
          prevCandidate: null,
        });
      }
    }

    return candidates;
  }

  private routeDistance(a: GeoPoint, b: GeoPoint): number {
    return haversine(a, b);
  }

  private gaussian(x: number, mu: number, sigma: number): number {
    return Math.exp(-0.5 * ((x - mu) / sigma) ** 2);
  }

  getPrevTimestamp(): number {
    return Date.now() - 100;
  }

  reset(): void {
    this.prevCandidates = [];
  }
}
