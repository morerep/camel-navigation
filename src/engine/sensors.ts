import type { Measurement, GeoPoint, WifiAP, CellTower } from '@/types/navigation';

export class WifiLocator {
  private readonly rssiAt1m = -40;
  private readonly pathLossExponent = 3.5;
  private readonly minAPs = 3;
  private readonly ransacIterations = 100;
  private readonly ransacThreshold = 30;

  locate(scan: WifiAP[], cellFix: GeoPoint | null): Measurement | null {
    if (scan.length < this.minAPs) return null;

    const anchors = scan.map(ap => ({
      ...ap,
      distance: this.rssiToDistance(ap.rssi),
    }));

    let bestPosition: GeoPoint | null = null;
    let bestInliers = 0;
    let bestSigma = 1000;

    for (let iter = 0; iter < this.ransacIterations; iter++) {
      const sample = this.shuffle([...anchors]).slice(0, 3);
      const position = this.trilaterate(sample);
      if (!position) continue;

      let inliers = 0;
      let totalError = 0;
      for (const ap of anchors) {
        const d = this.haversine(position, { lat: ap.lat, lng: ap.lng });
        const error = Math.abs(d - ap.distance);
        if (error < this.ransacThreshold) inliers++;
        totalError += error;
      }

      const sigma = totalError / anchors.length;
      if (inliers > bestInliers || (inliers === bestInliers && sigma < bestSigma)) {
        bestInliers = inliers;
        bestPosition = position;
        bestSigma = sigma;
      }
    }

    if (!bestPosition || bestInliers < this.minAPs) return null;

    if (cellFix) {
      const cellDist = this.haversine(bestPosition, cellFix);
      if (cellDist > 200) {
        bestSigma *= 3;
      }
    }

    return {
      type: 'wifi',
      position: bestPosition,
      sigmaM: Math.max(bestSigma, 15),
      timestamp: Date.now(),
      source: `wifi_${bestInliers}ap`,
      weight: Math.min(1, bestInliers / 6),
    };
  }

  private rssiToDistance(rssi: number): number {
    return Math.pow(10, (this.rssiAt1m - rssi) / (10 * this.pathLossExponent));
  }

  private trilaterate(aps: (WifiAP & { distance: number })[]): GeoPoint | null {
    if (aps.length < 3) return null;

    const [a, b, c] = aps;
    const latFactor = 111320;
    const lngFactor = 111320 * Math.cos(a.lat * Math.PI / 180);

    const bx = (b.lng - a.lng) * lngFactor;
    const by = (b.lat - a.lat) * latFactor;
    const cx = (c.lng - a.lng) * lngFactor;
    const cy = (c.lat - a.lat) * latFactor;

    const d = a.distance, e = b.distance, f = c.distance;

    const A = 2 * bx;
    const B = 2 * by;
    const C = d * d - e * e + bx * bx + by * by;
    const D = 2 * cx;
    const E = 2 * cy;
    const F = d * d - f * f + cx * cx + cy * cy;

    const denom = A * E - B * D;
    if (Math.abs(denom) < 1e-10) return null;

    const x = (C * E - B * F) / denom;
    const y = (A * F - C * D) / denom;

    return {
      lat: a.lat + y / latFactor,
      lng: a.lng + x / lngFactor,
    };
  }

  private haversine(a: GeoPoint, b: GeoPoint): number {
    const R = 6371000;
    const dLat = (b.lat - a.lat) * Math.PI / 180;
    const dLng = (b.lng - a.lng) * Math.PI / 180;
    const sinDLat2 = Math.sin(dLat / 2);
    const sinDLng2 = Math.sin(dLng / 2);
    const c = 2 * Math.asin(Math.sqrt(sinDLat2 * sinDLat2 + Math.cos(a.lat * Math.PI / 180) * Math.cos(b.lat * Math.PI / 180) * sinDLng2 * sinDLng2));
    return R * c;
  }

  private shuffle<T>(arr: T[]): T[] {
    for (let i = arr.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [arr[i], arr[j]] = [arr[j], arr[i]];
    }
    return arr;
  }
}

export class CellLocator {
  locate(towers: CellTower[]): Measurement | null {
    if (towers.length === 0) return null;

    let totalWeight = 0;
    let lat = 0, lng = 0;

    for (const tower of towers) {
      const weight = Math.pow(10, tower.signal / 30);
      lat += tower.lat * weight;
      lng += tower.lng * weight;
      totalWeight += weight;
    }

    if (totalWeight === 0) return null;

    lat /= totalWeight;
    lng /= totalWeight;

    const bestSignal = Math.max(...towers.map(t => t.signal));
    const sigmaM = bestSignal > -85 ? 500 : bestSignal > -95 ? 1000 : 2000;

    return {
      type: 'cell',
      position: { lat, lng },
      sigmaM,
      timestamp: Date.now(),
      source: `cell_${towers.length}towers`,
      weight: 0.3,
    };
  }
}

export class GpsAdapter {
  processReading(lat: number, lng: number, accuracy: number, speed?: number, heading?: number): Measurement {
    const urbanFactor = accuracy > 20 ? 1.5 : 1.0;
    const sigmaM = Math.max(accuracy, 5) * urbanFactor;

    return {
      type: 'gps',
      position: { lat, lng },
      heading: heading !== undefined ? heading : undefined,
      speed: speed !== undefined ? speed : undefined,
      sigmaM,
      sigmaHdg: heading !== undefined ? 15 : undefined,
      timestamp: Date.now(),
      source: 'gps',
      weight: accuracy < 10 ? 0.9 : accuracy < 30 ? 0.7 : 0.4,
    };
  }
}

export class CompassAdapter {
  private readonly magneticDeclination = 0.5;

  processHeading(magneticHeading: number, sigmaHdg: number = 30): Measurement {
    const trueHeading = (magneticHeading + this.magneticDeclination + 360) % 360;
    return {
      type: 'compass',
      heading: trueHeading,
      sigmaHdg: Math.max(sigmaHdg, 15),
      sigmaM: 1000,
      timestamp: Date.now(),
      source: 'compass',
      weight: 0.3,
    };
  }
}

export class ObdAdapter {
  processSpeed(kmh: number): Measurement {
    const ms = kmh / 3.6;
    return {
      type: 'obd',
      speed: ms,
      sigmaM: 0.16,
      timestamp: Date.now(),
      source: 'obd',
      weight: 0.95,
    };
  }
}

export class ImuAdapter {
  private zuptCount = 0;
  private readonly zuptThreshold = 0.3;
  private readonly zuptRequired = 5;

  processAccel(ax: number, ay: number, az: number): { isZUPT: boolean; measurement: Measurement | null } {
    const mag = Math.sqrt(ax * ax + ay * ay + az * az);

    const diff = Math.abs(mag - 9.81);
    if (diff < this.zuptThreshold) {
      this.zuptCount++;
    } else {
      this.zuptCount = 0;
    }

    const isZUPT = this.zuptCount >= this.zuptRequired;

    if (isZUPT) {
      return {
        isZUPT,
        measurement: {
          type: 'imu',
          speed: 0,
          sigmaM: 0.1,
          timestamp: Date.now(),
          source: 'imu_zupt',
          weight: 1.0,
        },
      };
    }

    return { isZUPT: false, measurement: null };
  }

  processGyro(_wx: number, _wy: number, wz: number): number {
    return wz;
  }
}
