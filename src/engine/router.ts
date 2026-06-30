// ============================================================================
// A* Router - Optimal pathfinding on road graph
// ============================================================================

import type { RoadGraph, RoadEdge, RoadNode, Route, Maneuver, GeoPoint } from '@/types/navigation';

function haversine(a: GeoPoint, b: GeoPoint): number {
  const R = 6371000;
  const dLat = (b.lat - a.lat) * Math.PI / 180;
  const dLng = (b.lng - a.lng) * Math.PI / 180;
  const sinDLat2 = Math.sin(dLat / 2);
  const sinDLng2 = Math.sin(dLng / 2);
  const c = 2 * Math.asin(Math.sqrt(sinDLat2 * sinDLat2 + Math.cos(a.lat * Math.PI / 180) * Math.cos(b.lat * Math.PI / 180) * sinDLng2 * sinDLng2));
  return R * c;
}

function roadClassWeight(rc: string): number {
  switch (rc) {
    case 'motorway': return 1.0;
    case 'trunk': return 1.1;
    case 'primary': return 1.2;
    case 'secondary': return 1.4;
    case 'tertiary': return 1.6;
    case 'residential': return 2.0;
    case 'service': return 2.5;
    default: return 2.0;
  }
}

function estimateEdgeDuration(edge: RoadEdge): number {
  const speedKmh = edge.speedLimit || 50;
  const weight = roadClassWeight(edge.roadClass);
  return (edge.distance / (speedKmh / 3.6)) * weight;
}

function generateManeuver(prevEdge: RoadEdge | null, currEdge: RoadEdge, nextEdge: RoadEdge | null): Maneuver['type'] {
  if (!prevEdge) return 'depart';
  if (!nextEdge) return 'arrive';
  
  const prevBearing = calculateBearing(
    prevEdge.waypoints[prevEdge.waypoints.length - 2] || { lat: 0, lng: 0 },
    prevEdge.waypoints[prevEdge.waypoints.length - 1]
  );
  const nextBearing = calculateBearing(
    nextEdge.waypoints[0],
    nextEdge.waypoints[1] || nextEdge.waypoints[0]
  );
  
  let angle = nextBearing - prevBearing;
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  
  if (Math.abs(angle) < 15) return 'straight';
  if (angle > 15 && angle < 45) return 'turn-slight-right';
  if (angle >= 45 && angle < 135) return 'turn-right';
  if (angle >= 135 && angle < 165) return 'turn-right';
  if (angle >= 165 || angle <= -165) return 'uturn';
  if (angle > -45 && angle < -15) return 'turn-slight-left';
  if (angle > -135 && angle <= -45) return 'turn-left';
  if (angle > -165 && angle <= -135) return 'turn-left';
  
  if (currEdge.name?.toLowerCase().includes('roundabout')) return 'roundabout';
  
  return 'straight';
}

function calculateBearing(from: GeoPoint, to: GeoPoint): number {
  const dLng = (to.lng - from.lng) * Math.PI / 180;
  const lat1 = from.lat * Math.PI / 180;
  const lat2 = to.lat * Math.PI / 180;
  const y = Math.sin(dLng) * Math.cos(lat2);
  const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLng);
  let bearing = Math.atan2(y, x) * 180 / Math.PI;
  return ((bearing % 360) + 360) % 360;
}

function maneuverInstruction(maneuver: Maneuver['type'], roadName: string): string {
  const road = roadName || 'unnamed road';
  switch (maneuver) {
    case 'depart': return `Head northeast on ${road}`;
    case 'arrive': return `Arrive at destination`;
    case 'straight': return `Continue on ${road}`;
    case 'turn-left': return `Turn left onto ${road}`;
    case 'turn-right': return `Turn right onto ${road}`;
    case 'turn-slight-left': return `Slight left onto ${road}`;
    case 'turn-slight-right': return `Slight right onto ${road}`;
    case 'uturn': return `Make a U-turn`;
    case 'roundabout': return `Enter the roundabout`;
    case 'merge': return `Merge onto ${road}`;
    case 'exit': return `Take the exit`;
    case 'ramp-left': return `Take the ramp on the left`;
    case 'ramp-right': return `Take the ramp on the right`;
    default: return `Continue on ${road}`;
  }
}

export class AStarRouter {
  private graph: RoadGraph;
  constructor(graph: RoadGraph) { this.graph = graph; }

  findRoute(from: GeoPoint, to: GeoPoint): Route | null {
    const fromNode = this.findNearestNode(from);
    const toNode = this.findNearestNode(to);
    if (!fromNode || !toNode) return null;

    const openSet = new Set<number>();
    const closedSet = new Set<number>();
    const gScore = new Map<number, number>();
    const fScore = new Map<number, number>();
    const cameFromEdge = new Map<number, number>();
    const cameFromNode = new Map<number, number>();

    openSet.add(fromNode.id);
    gScore.set(fromNode.id, 0);
    fScore.set(fromNode.id, haversine(fromNode, toNode));

    while (openSet.size > 0) {
      let current = -1;
      let lowestF = Infinity;
      for (const nodeId of openSet) {
        const f = fScore.get(nodeId) || Infinity;
        if (f < lowestF) { lowestF = f; current = nodeId; }
      }

      if (current === toNode.id) {
        return this.reconstructRoute(cameFromEdge, cameFromNode, toNode.id, from, to);
      }

      openSet.delete(current);
      closedSet.add(current);

      const edgeIds = this.graph.adjacency.get(current) || [];
      for (const edgeId of edgeIds) {
        const edge = this.graph.edges.get(edgeId);
        if (!edge) continue;

        const neighbor = edge.to;
        if (closedSet.has(neighbor)) continue;

        const tentativeG = (gScore.get(current) || Infinity) + estimateEdgeDuration(edge);

        if (!openSet.has(neighbor)) {
          openSet.add(neighbor);
        } else if (tentativeG >= (gScore.get(neighbor) || Infinity)) {
          continue;
        }

        cameFromEdge.set(neighbor, edgeId);
        cameFromNode.set(neighbor, current);
        gScore.set(neighbor, tentativeG);
        const neighborNode = this.graph.nodes.get(neighbor)!;
        fScore.set(neighbor, tentativeG + haversine(neighborNode, toNode) / 13.89);
      }
    }

    return null;
  }

  private findNearestNode(point: GeoPoint): RoadNode | null {
    let nearest: RoadNode | null = null;
    let minDist = Infinity;
    for (const [, node] of this.graph.nodes) {
      const d = haversine(point, node);
      if (d < minDist) { minDist = d; nearest = node; }
    }
    return nearest;
  }

  private reconstructRoute(
    cameFromEdge: Map<number, number>,
    cameFromNode: Map<number, number>,
    endNodeId: number,
    from: GeoPoint,
    to: GeoPoint
  ): Route {
    const edges: RoadEdge[] = [];
    const nodes: RoadNode[] = [];
    let current = endNodeId;

    while (cameFromEdge.has(current)) {
      const edgeId = cameFromEdge.get(current)!;
      const edge = this.graph.edges.get(edgeId)!;
      edges.unshift(edge);
      nodes.unshift(this.graph.nodes.get(edge.from)!);
      current = cameFromNode.get(current)!;
    }
    nodes.push(this.graph.nodes.get(endNodeId)!);

    const waypoints: GeoPoint[] = [from];
    for (const edge of edges) {
      for (const wp of edge.waypoints) waypoints.push(wp);
    }
    waypoints.push(to);

    const maneuvers: Maneuver[] = [];
    let totalDistance = 0;
    let totalDuration = 0;

    for (let i = 0; i < edges.length; i++) {
      totalDistance += edges[i].distance;
      totalDuration += estimateEdgeDuration(edges[i]);

      const prevEdge = i > 0 ? edges[i - 1] : null;
      const nextEdge = i < edges.length - 1 ? edges[i + 1] : null;
      const type = generateManeuver(prevEdge, edges[i], nextEdge);
      const distToHere = edges.slice(i).reduce((sum, e) => sum + e.distance, 0);

      if (type !== 'straight' || i === 0 || i === edges.length - 1) {
        maneuvers.push({
          type,
          instruction: maneuverInstruction(type, edges[i].name),
          distance: distToHere,
          point: edges[i].waypoints[0] || from,
          edgeIndex: i,
        });
      }
    }

    return { edges, nodes, waypoints, distance: totalDistance, duration: totalDuration, maneuvers };
  }
}
