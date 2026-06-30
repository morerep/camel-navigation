import type { Route, Maneuver, PoseEstimate, MatchedSegment, GuidanceEvent, NavigationState } from '@/types/navigation';

export class GuidanceEngine {
  private route: Route | null = null;
  private announcedManeuvers = new Set<string>();
  private offRouteCount = 0;
  private readonly offRouteThreshold = 3;

  setRoute(route: Route | null): void {
    this.route = route;
    this.announcedManeuvers.clear();
    this.offRouteCount = 0;
  }

  update(pose: PoseEstimate, matchedSegment: MatchedSegment | null): NavigationState {
    if (!this.route) {
      return {
        route: null,
        pose,
        matchedSegment,
        nextManeuver: null,
        distanceToNext: 0,
        distanceRemaining: 0,
        timeRemaining: 0,
        offRoute: false,
        guidance: null,
      };
    }

    const { edgeIndex, distanceAlong, distanceRemaining } = this.findRouteProgress(matchedSegment);
    const nextManeuver = this.findNextManeuver(edgeIndex);
    const distanceToNext = nextManeuver
      ? this.distanceToManeuver(edgeIndex, distanceAlong, nextManeuver)
      : 0;

    const offRoute = this.checkOffRoute(matchedSegment);
    const guidance = this.generateGuidance(nextManeuver, distanceToNext, edgeIndex);

    const avgSpeed = Math.max(pose.speed, 5);
    const timeRemaining = distanceRemaining / avgSpeed;

    return {
      route: this.route,
      pose,
      matchedSegment,
      nextManeuver,
      distanceToNext,
      distanceRemaining,
      timeRemaining,
      offRoute,
      guidance,
    };
  }

  private findRouteProgress(matched: MatchedSegment | null): { edgeIndex: number; distanceAlong: number; distanceRemaining: number } {
    if (!this.route || !matched) {
      return { edgeIndex: 0, distanceAlong: 0, distanceRemaining: this.route?.distance || 0 };
    }

    let edgeIndex = 0;
    let distanceAlong = 0;
    let found = false;

    for (let i = 0; i < this.route.edges.length; i++) {
      if (this.route.edges[i].id === matched.edgeId) {
        edgeIndex = i;
        distanceAlong = matched.arcPos;
        found = true;
        break;
      }
    }

    let distanceRemaining = 0;
    for (let i = edgeIndex; i < this.route.edges.length; i++) {
      distanceRemaining += this.route.edges[i].distance;
    }
    distanceRemaining -= distanceAlong;

    return { edgeIndex, distanceAlong, distanceRemaining: Math.max(0, distanceRemaining) };
  }

  private findNextManeuver(currentEdgeIndex: number): Maneuver | null {
    if (!this.route) return null;
    for (const maneuver of this.route.maneuvers) {
      if (maneuver.edgeIndex > currentEdgeIndex) {
        return maneuver;
      }
    }
    return null;
  }

  private distanceToManeuver(currentEdgeIndex: number, distanceAlong: number, maneuver: Maneuver): number {
    if (!this.route) return 0;
    let dist = 0;
    for (let i = currentEdgeIndex; i < maneuver.edgeIndex; i++) {
      dist += this.route.edges[i]?.distance || 0;
    }
    dist -= distanceAlong;
    return Math.max(0, dist);
  }

  private checkOffRoute(matched: MatchedSegment | null): boolean {
    if (!matched || matched.confidence < 0.3) {
      this.offRouteCount++;
    } else {
      this.offRouteCount = Math.max(0, this.offRouteCount - 1);
    }
    return this.offRouteCount >= this.offRouteThreshold;
  }

  private generateGuidance(maneuver: Maneuver | null, distanceToNext: number, edgeIndex: number): GuidanceEvent | null {
    if (!maneuver) {
      if (this.route && edgeIndex >= this.route.edges.length - 1) {
        return {
          type: 'arriving',
          message: 'You have arrived at your destination',
          severity: 'info',
        };
      }
      return null;
    }

    const key = `${String(edgeIndex)}-${maneuver.type}`;

    if (distanceToNext < 50 && !this.announcedManeuvers.has(key + '-now')) {
      this.announcedManeuvers.add(key + '-now');
      return {
        type: 'maneuver_now',
        message: maneuver.instruction,
        severity: 'critical',
        distance: distanceToNext,
      };
    }

    if (distanceToNext < 200 && !this.announcedManeuvers.has(key + '-200')) {
      this.announcedManeuvers.add(key + '-200');
      return {
        type: 'maneuver_approach',
        message: `In 200 meters, ${maneuver.instruction.toLowerCase()}`,
        severity: 'warning',
        distance: distanceToNext,
      };
    }

    if (distanceToNext < 500 && !this.announcedManeuvers.has(key + '-500')) {
      this.announcedManeuvers.add(key + '-500');
      return {
        type: 'maneuver_approach',
        message: `In 500 meters, ${maneuver.instruction.toLowerCase()}`,
        severity: 'info',
        distance: distanceToNext,
      };
    }

    return null;
  }

  requestReroute(): GuidanceEvent {
    return {
      type: 'reroute',
      message: 'Recalculating route...',
      severity: 'warning',
    };
  }
}
