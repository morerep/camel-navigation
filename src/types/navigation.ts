export interface GeoPoint {
  lat: number;
  lng: number;
}

export interface PoseEstimate {
  position: GeoPoint;
  heading: number;
  speed: number;
  sigmaM: number;
  sigmaHdg: number;
  health: 'good' | 'fair' | 'poor' | 'diverged' | 'corrupt';
  timestamp: number;
  source: string;
}

export interface Measurement {
  type: 'gps' | 'wifi' | 'cell' | 'imu' | 'compass' | 'obd' | 'map_match' | 'slam' | 'assisted';
  position?: GeoPoint;
  heading?: number;
  speed?: number;
  sigmaM: number;
  sigmaHdg?: number;
  timestamp: number;
  source: string;
  weight: number;
}

export interface RoadNode {
  id: number;
  lat: number;
  lng: number;
}

export interface RoadEdge {
  id: number;
  from: number;
  to: number;
  waypoints: GeoPoint[];
  distance: number;
  speedLimit: number;
  roadClass: 'motorway' | 'trunk' | 'primary' | 'secondary' | 'tertiary' | 'residential' | 'service' | 'unclassified';
  name: string;
  oneWay: boolean;
  lanes: number;
}

export interface RoadGraph {
  nodes: Map<number, RoadNode>;
  edges: Map<number, RoadEdge>;
  adjacency: Map<number, number[]>;
}

export interface MatchedSegment {
  edgeId: number;
  arcPos: number;
  confidence: number;
  lateralError: number;
}

export interface Route {
  edges: RoadEdge[];
  nodes: RoadNode[];
  waypoints: GeoPoint[];
  distance: number;
  duration: number;
  maneuvers: Maneuver[];
}

export interface Maneuver {
  type: 'straight' | 'turn-left' | 'turn-right' | 'turn-slight-left' | 'turn-slight-right' | 
        'uturn' | 'roundabout' | 'merge' | 'exit' | 'arrive' | 'depart' | 'ramp-left' | 'ramp-right';
  instruction: string;
  distance: number;
  point: GeoPoint;
  edgeIndex: number;
  exitNumber?: number;
}

export interface NavigationState {
  route: Route | null;
  pose: PoseEstimate;
  matchedSegment: MatchedSegment | null;
  nextManeuver: Maneuver | null;
  distanceToNext: number;
  distanceRemaining: number;
  timeRemaining: number;
  offRoute: boolean;
  guidance: GuidanceEvent | null;
}

export interface GuidanceEvent {
  type: 'maneuver_approach' | 'maneuver_now' | 'lane_guidance' | 'speed_limit' | 
        'traffic' | 'anomaly_ahead' | 'arriving' | 'reroute';
  message: string;
  severity: 'info' | 'warning' | 'critical';
  distance?: number;
}

export interface WifiAP {
  bssid: string;
  ssid: string;
  lat: number;
  lng: number;
  rssi: number;
  frequency: number;
  lastSeen: number;
}

export interface CellTower {
  cellId: number;
  lac: number;
  mcc: number;
  mnc: number;
  lat: number;
  lng: number;
  signal: number;
  type: 'gsm' | 'lte' | 'nr';
}

export interface POI {
  id: string;
  name: string;
  category: string;
  lat: number;
  lng: number;
  address?: string;
  rating?: number;
}

export interface RoadSignature {
  edgeId: number;
  arcPos: number;
  magX: number;
  magY: number;
  magZ: number;
  accelZ: number;
  accelLateral: number;
  gyroYaw: number;
  quality: number;
  sampleCount: number;
}

export interface RoadAnomaly {
  id: string;
  edgeId: number;
  arcPos: number;
  type: 'pothole' | 'speed_bump' | 'rough_surface' | 'grade_change' | 'bridge' | 'tunnel' | 'steel_structure';
  severity: number;
  confidence: number;
  firstSeen: number;
  lastSeen: number;
  passCount: number;
  lat: number;
  lng: number;
}

export interface NavigationConfig {
  dubaiCenter: GeoPoint;
  dubaiBounds: [[number, number], [number, number]];
  defaultZoom: number;
  minZoom: 10;
  maxZoom: 20;
  rerouteThresholdM: number;
  maneuverAnnouncementDistances: number[];
  ekfProcessNoise: number;
  mapMatchConfidenceThreshold: number;
}

export const DEFAULT_CONFIG: NavigationConfig = {
  dubaiCenter: { lat: 25.2048, lng: 55.2708 },
  dubaiBounds: [[54.8, 24.8], [55.6, 25.4]],
  defaultZoom: 14,
  minZoom: 10,
  maxZoom: 20,
  rerouteThresholdM: 50,
  maneuverAnnouncementDistances: [500, 200, 50],
  ekfProcessNoise: 0.1,
  mapMatchConfidenceThreshold: 0.6,
};
