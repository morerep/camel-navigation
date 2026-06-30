import type { RoadAnomaly, GeoPoint, MatchedSegment } from '@/types/navigation';

interface AccelSample {
  ax: number;
  ay: number;
  az: number;
  timestamp: number;
  speed: number;
  position: GeoPoint;
  matchedSegment: MatchedSegment | null;
}

export class AnomalyDetector {
  private window: AccelSample[] = [];
  private readonly windowSize = 30;
  private detectedAnomalies: RoadAnomaly[] = [];
  private readonly gravity = 9.81;
  private readonly potholeThreshold = 11.77;
  private readonly potholeDurationMax = 200;
  private readonly speedBumpThreshold = 6.0;
  private readonly roughSurfaceStd = 3.0;

  ingest(sample: AccelSample): RoadAnomaly | null {
    this.window.push(sample);
    if (this.window.length > this.windowSize) {
      this.window.shift();
    }

    if (this.window.length < 5) return null;
    if (sample.speed < 3) return null;

    const pothole = this.detectPothole();
    if (pothole) {
      this.detectedAnomalies.push(pothole);
      return pothole;
    }

    const speedBump = this.detectSpeedBump();
    if (speedBump) {
      this.detectedAnomalies.push(speedBump);
      return speedBump;
    }

    const roughSurface = this.detectRoughSurface();
    if (roughSurface) {
      this.detectedAnomalies.push(roughSurface);
      return roughSurface;
    }

    return null;
  }

  private detectPothole(): RoadAnomaly | null {
    const recent = this.window.slice(-10);
    if (recent.length < 5) return null;

    let peakAccel = 0;
    let peakTime = 0;
    const zVals = recent.map(s => Math.abs(s.az - this.gravity));
    const baseline = this.median(zVals);

    for (const s of recent) {
      const z = Math.abs(s.az - this.gravity);
      if (z > peakAccel) {
        peakAccel = z;
        peakTime = s.timestamp;
      }
    }

    if (peakAccel - baseline < this.potholeThreshold) return null;

    const duration = this.durationAboveThreshold(zVals, baseline + this.potholeThreshold * 0.7);
    if (duration > this.potholeDurationMax) return null;

    const speed = recent[recent.length - 1].speed;
    if (speed < 5) return null;

    const pos = recent[Math.floor(recent.length / 2)].position;
    const matched = recent[Math.floor(recent.length / 2)].matchedSegment;

    return {
      id: `anom_${Date.now()}_pothole`,
      edgeId: matched?.edgeId || 0,
      arcPos: matched?.arcPos || 0,
      type: 'pothole',
      severity: Math.min(1, (peakAccel - baseline) / 20),
      confidence: 0.7,
      firstSeen: peakTime,
      lastSeen: peakTime,
      passCount: 1,
      lat: pos.lat,
      lng: pos.lng,
    };
  }

  private detectSpeedBump(): RoadAnomaly | null {
    const recent = this.window.slice(-15);
    if (recent.length < 10) return null;

    const zVals = recent.map(s => Math.abs(s.az - this.gravity));
    const baseline = this.median(zVals);

    const aboveThreshold = zVals.map(z => z > this.speedBumpThreshold);
    let peakCount = 0;
    let inPeak = false;
    for (const above of aboveThreshold) {
      if (above && !inPeak) { peakCount++; inPeak = true; }
      else if (!above) inPeak = false;
    }

    if (peakCount < 2) return null;

    const severity = Math.min(1, (Math.max(...zVals) - baseline) / 10);
    const pos = recent[Math.floor(recent.length / 2)].position;
    const matched = recent[Math.floor(recent.length / 2)].matchedSegment;

    return {
      id: `anom_${Date.now()}_bump`,
      edgeId: matched?.edgeId || 0,
      arcPos: matched?.arcPos || 0,
      type: 'speed_bump',
      severity,
      confidence: 0.6,
      firstSeen: recent[0].timestamp,
      lastSeen: recent[recent.length - 1].timestamp,
      passCount: 1,
      lat: pos.lat,
      lng: pos.lng,
    };
  }

  private detectRoughSurface(): RoadAnomaly | null {
    const recent = this.window;
    if (recent.length < this.windowSize * 0.8) return null;

    const zVals = recent.map(s => Math.abs(s.az - this.gravity));
    const std = this.stdDev(zVals);

    if (std < this.roughSurfaceStd) return null;

    const pos = recent[Math.floor(recent.length / 2)].position;
    const matched = recent[Math.floor(recent.length / 2)].matchedSegment;

    return {
      id: `anom_${Date.now()}_rough`,
      edgeId: matched?.edgeId || 0,
      arcPos: matched?.arcPos || 0,
      type: 'rough_surface',
      severity: Math.min(1, std / 8),
      confidence: 0.5,
      firstSeen: recent[0].timestamp,
      lastSeen: recent[recent.length - 1].timestamp,
      passCount: 1,
      lat: pos.lat,
      lng: pos.lng,
    };
  }

  private median(arr: number[]): number {
    const sorted = [...arr].sort((a, b) => a - b);
    const mid = Math.floor(sorted.length / 2);
    return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
  }

  private stdDev(arr: number[]): number {
    const mean = arr.reduce((a, b) => a + b, 0) / arr.length;
    return Math.sqrt(arr.reduce((sq, n) => sq + (n - mean) ** 2, 0) / arr.length);
  }

  private durationAboveThreshold(values: number[], threshold: number): number {
    let maxDuration = 0;
    let currentDuration = 0;
    for (const v of values) {
      if (v > threshold) {
        currentDuration += 100;
        maxDuration = Math.max(maxDuration, currentDuration);
      } else {
        currentDuration = 0;
      }
    }
    return maxDuration;
  }

  getAnomalies(): RoadAnomaly[] {
    return [...this.detectedAnomalies];
  }

  clear(): void {
    this.window = [];
    this.detectedAnomalies = [];
  }

  static promoteToConfirmed(anomalies: RoadAnomaly[], newAnomaly: RoadAnomaly): RoadAnomaly | null {
    const nearby = anomalies.filter(a =>
      a.type === newAnomaly.type &&
      Math.abs(a.arcPos - newAnomaly.arcPos) < 10 &&
      a.edgeId === newAnomaly.edgeId
    );

    if (nearby.length >= 2) {
      const allSeverities = [...nearby.map(a => a.severity), newAnomaly.severity];
      const medianSeverity = allSeverities.sort((a, b) => a - b)[Math.floor(allSeverities.length / 2)];

      return {
        ...newAnomaly,
        confidence: Math.min(1, newAnomaly.confidence + 0.2 * nearby.length),
        severity: medianSeverity,
        passCount: nearby.length + 1,
      };
    }

    return null;
  }
}
