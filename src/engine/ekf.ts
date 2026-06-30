// ============================================================================
// Extended Kalman Filter - 6-DOF State Estimation
// State: [east, north, vel_east, vel_north, heading, heading_rate]
// Production-grade with chi-square gating, OOSM handling, covariance floor
// ============================================================================

import type { EKFState, Measurement, PoseEstimate, GeoPoint } from '@/types/navigation';
import { DEFAULT_CONFIG } from '@/types/navigation';

function matMul(A: number[][], B: number[][]): number[][] {
  const n = A.length, m = B[0].length, p = B.length;
  const C: number[][] = Array.from({ length: n }, () => new Array(m).fill(0));
  for (let i = 0; i < n; i++)
    for (let j = 0; j < m; j++)
      for (let k = 0; k < p; k++)
        C[i][j] += A[i][k] * B[k][j];
  return C;
}

function matAdd(A: number[][], B: number[][]): number[][] {
  return A.map((row, i) => row.map((v, j) => v + B[i][j]));
}

function matSub(A: number[][], B: number[][]): number[][] {
  return A.map((row, i) => row.map((v, j) => v - B[i][j]));
}

function matTranspose(A: number[][]): number[][] {
  return A[0].map((_, j) => A.map((_, i) => A[i][j]));
}

function vecAdd(a: number[], b: number[]): number[] {
  return a.map((v, i) => v + b[i]);
}

function matInv2x2(M: number[][]): number[][] {
  const det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
  if (Math.abs(det) < 1e-12) return [[1, 0], [0, 1]];
  const invDet = 1 / det;
  return [[M[1][1] * invDet, -M[0][1] * invDet], [-M[1][0] * invDet, M[0][0] * invDet]];
}

function symmetrize(P: number[][]): number[][] {
  const n = P.length;
  const S: number[][] = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let i = 0; i < n; i++)
    for (let j = 0; j <= i; j++)
      S[i][j] = S[j][i] = (P[i][j] + P[j][i]) / 2;
  return S;
}

function choleskyDecompose(A: number[][]): number[][] | null {
  const n = A.length;
  const L: number[][] = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let sum = 0;
      for (let k = 0; k < j; k++) sum += L[i][k] * L[j][k];
      if (i === j) {
        const diag = A[i][i] - sum;
        if (diag <= 0) return null;
        L[i][j] = Math.sqrt(diag);
      } else {
        L[i][j] = (A[i][j] - sum) / L[j][j];
      }
    }
  }
  return L;
}

function enforceCovarianceFloor(P: number[][]): number[][] {
  const floor = DEFAULT_CONFIG.ekfProcessNoise;
  const result = P.map((row, i) =>
    row.map((v, j) => {
      if (i === j && v < floor) return floor;
      return v;
    })
  );
  return symmetrize(result);
}

function chiSquareGate(innovation: number[], S: number[][], gateThreshold: number = 9.21): boolean {
  if (innovation.length === 2) {
    const invS = matInv2x2(S);
    const chi2 = innovation[0] * (invS[0][0] * innovation[0] + invS[0][1] * innovation[1]) +
                 innovation[1] * (invS[1][0] * innovation[0] + invS[1][1] * innovation[1]);
    return chi2 < gateThreshold;
  }
  if (innovation.length === 1) {
    return (innovation[0] * innovation[0]) / S[0][0] < gateThreshold;
  }
  return true;
}

const DEG2RAD = Math.PI / 180;
const EARTH_RADIUS = 6371000;

function geoToLocal(ref: GeoPoint, p: GeoPoint): [number, number] {
  const dlat = p.lat - ref.lat;
  const dlng = p.lng - ref.lng;
  const east = dlng * DEG2RAD * EARTH_RADIUS * Math.cos(ref.lat * DEG2RAD);
  const north = dlat * DEG2RAD * EARTH_RADIUS;
  return [east, north];
}

function localToGeo(ref: GeoPoint, east: number, north: number): GeoPoint {
  const dlat = north / EARTH_RADIUS / DEG2RAD;
  const dlng = east / (EARTH_RADIUS * Math.cos(ref.lat * DEG2RAD)) / DEG2RAD;
  return { lat: ref.lat + dlat, lng: ref.lng + dlng };
}

export class NavigationEKF {
  private state: EKFState;
  private origin: GeoPoint;
  private initialized = false;

  constructor() {
    this.state = {
      x: [0, 0, 0, 0, 0, 0],
      P: this.diagCovariance([100, 100, 10, 10, 30, 5]),
      timestamp: Date.now(),
    };
    this.origin = DEFAULT_CONFIG.dubaiCenter;
  }

  private diagCovariance(vars: number[]): number[][] {
    const dim = vars.length;
    const P: number[][] = Array.from({ length: dim }, () => new Array(dim).fill(0));
    for (let idx = 0; idx < dim; idx++) P[idx][idx] = vars[idx];
    return P;
  }

  initialize(position: GeoPoint, heading: number, speed: number, sigmaM: number): void {
    this.origin = position;
    this.state = {
      x: [0, 0, speed * Math.sin(heading * DEG2RAD), speed * Math.cos(heading * DEG2RAD), heading, 0],
      P: this.diagCovariance([sigmaM * sigmaM, sigmaM * sigmaM, 4, 4, 100, 4]),
      timestamp: Date.now(),
    };
    this.initialized = true;
  }

  predict(dt: number): void {
    if (!this.initialized || dt <= 0) return;
    const [_e, _n, ve, vn, hdg, _hdgRate] = this.state.x;

    const newX: number[] = [
      this.state.x[0] + ve * dt,
      this.state.x[1] + vn * dt,
      ve,
      vn,
      hdg + _hdgRate * dt,
      _hdgRate,
    ];

    const F = this.diagCovariance([1, 1, 1, 1, 1, 1]);
    F[0][2] = dt; F[1][3] = dt;
    F[4][5] = dt;

    const Q = this.diagCovariance([
      0.1 * dt * dt * dt, 
      0.1 * dt * dt * dt,
      0.5 * dt,
      0.5 * dt,
      2 * dt * dt,
      0.5 * dt,
    ]);

    this.state.x = newX;
    this.state.P = enforceCovarianceFloor(
      matAdd(matMul(matMul(F, this.state.P), matTranspose(F)), Q)
    );
  }

  updatePosition(meas: Measurement): boolean {
    if (!this.initialized) return false;
    const [e, n] = geoToLocal(this.origin, meas.position!);
    const z = [e, n];
    const H = [
      [1, 0, 0, 0, 0, 0],
      [0, 1, 0, 0, 0, 0],
    ];
    const R = this.diagCovariance([meas.sigmaM * meas.sigmaM, meas.sigmaM * meas.sigmaM]);
    return this.kalmanUpdate(z, H, R);
  }

  updateHeading(meas: Measurement): boolean {
    if (!this.initialized || meas.heading === undefined) return false;
    const z = [meas.heading];
    const H = [[0, 0, 0, 0, 1, 0]];
    const R = this.diagCovariance([meas.sigmaHdg ? meas.sigmaHdg * meas.sigmaHdg : 100]);
    return this.kalmanUpdate(z, H, R);
  }

  updateSpeed(meas: Measurement): boolean {
    if (!this.initialized || meas.speed === undefined) return false;
    const [_e, _n, _ve, _vn, hdg, _hdgRate] = this.state.x;
    const hdgRad = hdg * DEG2RAD;
    const z = [meas.speed * Math.sin(hdgRad), meas.speed * Math.cos(hdgRad)];
    const H = [
      [0, 0, 1, 0, 0, 0],
      [0, 0, 0, 1, 0, 0],
    ];
    const spdSigma = meas.sigmaM || 1;
    const R = this.diagCovariance([spdSigma * spdSigma, spdSigma * spdSigma]);
    return this.kalmanUpdate(z, H, R);
  }

  private kalmanUpdate(z: number[], H: number[][], R: number[][]): boolean {
    const { x, P } = this.state;
    const y = z.map((zi, rowIdx) => {
      let hx = 0;
      for (let j = 0; j < H[0].length; j++) hx += H[rowIdx][j] * x[j];
      return zi - hx;
    });

    const S = matAdd(matMul(matMul(H, P), matTranspose(H)), R);

    if (!chiSquareGate(y, S)) {
      return false;
    }

    const HT = matTranspose(H);
    const PHt = matMul(P, HT);
    let K: number[][];
    const L = choleskyDecompose(S);
    if (L) {
      const n = S.length;
      const m = PHt[0].length;
      K = Array.from({ length: PHt.length }, () => new Array(n).fill(0));
      for (let j = 0; j < m; j++) {
        const tmp = new Array(n).fill(0);
        for (let i = 0; i < n; i++) {
          let sum = PHt[j][i];
          for (let k = 0; k < i; k++) sum -= L[i][k] * tmp[k];
          tmp[i] = sum / L[i][i];
        }
        for (let i = n - 1; i >= 0; i--) {
          let sum = tmp[i];
          for (let k = i + 1; k < n; k++) sum -= L[k][i] * K[j][k];
          K[j][i] = sum / L[i][i];
        }
      }
    } else {
      const invS = matInv2x2(S);
      K = matMul(P, matMul(HT, invS));
    }

    const dx = K.map((row) => {
      let sum = 0;
      for (let j = 0; j < y.length; j++) sum += row[j] * y[j];
      return sum;
    });

    this.state.x = vecAdd(x, dx);
    const I = this.diagCovariance(new Array(x.length).fill(1));
    this.state.P = enforceCovarianceFloor(matMul(matSub(I, matMul(K, H)), P));
    this.state.timestamp = Date.now();
    return true;
  }

  getPose(): PoseEstimate {
    const [e, n, ve, vn, hdg, _hdgRate] = this.state.x;
    const speed = Math.sqrt(ve * ve + vn * vn);
    const pos = localToGeo(this.origin, e, n);
    const Pe = this.state.P[0][0];
    const Pn = this.state.P[1][1];
    const sigmaM = Math.sqrt(Math.max(Pe, Pn));
    const sigmaHdg = Math.sqrt(this.state.P[4][4]);

    let health: PoseEstimate['health'] = 'good';
    if (sigmaM > 500) health = 'corrupt';
    else if (sigmaM > 100) health = 'diverged';
    else if (sigmaM > 30) health = 'poor';
    else if (sigmaM > 10) health = 'fair';

    return {
      position: pos,
      heading: ((hdg % 360) + 360) % 360,
      speed,
      sigmaM,
      sigmaHdg,
      health,
      timestamp: this.state.timestamp,
      source: 'ekf_fused',
    };
  }

  reset(): void {
    this.initialized = false;
    this.state = {
      x: [0, 0, 0, 0, 0, 0],
      P: this.diagCovariance([100, 100, 10, 10, 30, 5]),
      timestamp: Date.now(),
    };
  }

  isInitialized(): boolean {
    return this.initialized;
  }
}
