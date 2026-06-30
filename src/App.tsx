import { useState, useEffect, useRef, useCallback } from 'react';
import Map from './components/Map';
import SearchPanel from './components/SearchPanel';
import DriveHUD from './components/DriveHUD';
import ManeuverCard from './components/ManeuverCard';
import Speedometer from './components/Speedometer';
import BottomBar from './components/BottomBar';
import { NavigationEKF } from './engine/ekf';
import { AStarRouter } from './engine/router';
import { HMMMapMatcher } from './engine/matcher';
import { GuidanceEngine } from './engine/guidance';
import { AnomalyDetector } from './engine/anomalies';
import { GpsAdapter } from './engine/sensors';
import { buildDubaiRoadGraph } from './data/dubaiRoads';
import { dubaiPOIs } from './data/dubaiPOIs';
import type { PoseEstimate, Route, NavigationState, RoadAnomaly, POI, GeoPoint } from './types/navigation';
import { Navigation, Locate, Shield, X, Menu, AlertTriangle } from 'lucide-react';

const SIMULATION_SPEED = 15;
const TICK_INTERVAL = 100;
const DUBAI_CENTER: GeoPoint = { lat: 25.2048, lng: 55.2708 };

export default function App() {
  const [screen, setScreen] = useState<'map' | 'navigating' | 'arrived'>('map');
  const [route, setRoute] = useState<Route | null>(null);
  const [pose, setPose] = useState<PoseEstimate>({
    position: DUBAI_CENTER,
    heading: 45,
    speed: 0,
    sigmaM: 5,
    sigmaHdg: 2,
    health: 'good',
    timestamp: Date.now(),
    source: 'gps',
  });
  const [navState, setNavState] = useState<NavigationState | null>(null);
  const [anomalies, setAnomalies] = useState<RoadAnomaly[]>([]);
  const [shieldMode, setShieldMode] = useState(false);
  const [searchOpen, setSearchOpen] = useState(false);
  const [selectedPOI, setSelectedPOI] = useState<POI | null>(null);
  const [mapStyle, setMapStyle] = useState<'dark' | 'light' | 'satellite'>('dark');
  const [menuOpen, setMenuOpen] = useState(false);
  const [showAnomalyAlert, setShowAnomalyAlert] = useState<RoadAnomaly | null>(null);

  const ekfRef = useRef(new NavigationEKF());
  const routerRef = useRef(new AStarRouter(buildDubaiRoadGraph()));
  const matcherRef = useRef(new HMMMapMatcher(buildDubaiRoadGraph()));
  const guidanceRef = useRef(new GuidanceEngine());
  const anomalyRef = useRef(new AnomalyDetector());
  const gpsRef = useRef(new GpsAdapter());
  
  const simIndexRef = useRef(0);
  const navTickRef = useRef<number | null>(null);
  const lastTickRef = useRef(Date.now());

  useEffect(() => {
    ekfRef.current.initialize(DUBAI_CENTER, 45, 0, 5);
  }, []);

  const startNavigation = useCallback((destination: POI) => {
    const destPoint = { lat: destination.lat, lng: destination.lng };
    const currentPose = ekfRef.current.getPose();
    const newRoute = routerRef.current.findRoute(currentPose.position, destPoint);
    
    if (!newRoute) {
      alert('No route found. Try a different destination.');
      return;
    }

    setRoute(newRoute);
    guidanceRef.current.setRoute(newRoute);
    matcherRef.current.reset();
    anomalyRef.current.clear();
    simIndexRef.current = 0;
    setAnomalies([]);
    setScreen('navigating');
    setSearchOpen(false);
    setSelectedPOI(destination);

    lastTickRef.current = Date.now();
    const tick = () => {
      const now = Date.now();
      const dt = (now - lastTickRef.current) / 1000;
      lastTickRef.current = now;

      if (newRoute.waypoints.length > 1) {
        const idx = Math.min(simIndexRef.current, newRoute.waypoints.length - 1);
        const wp = newRoute.waypoints[idx];
        
        let heading = pose.heading;
        if (idx < newRoute.waypoints.length - 1) {
          const next = newRoute.waypoints[idx + 1];
          const dLng = (next.lng - wp.lng) * Math.PI / 180;
          const lat1 = wp.lat * Math.PI / 180;
          const lat2 = next.lat * Math.PI / 180;
          const y = Math.sin(dLng) * Math.cos(lat2);
          const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLng);
          heading = (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
        }

        const gpsMeas = gpsRef.current.processReading(
          wp.lat, wp.lng, 
          shieldMode ? 200 : 8,
          SIMULATION_SPEED,
          heading
        );

        ekfRef.current.predict(dt);
        ekfRef.current.updatePosition(gpsMeas);
        if (gpsMeas.heading !== undefined) {
          ekfRef.current.updateHeading({
            type: 'gps',
            heading: gpsMeas.heading,
            sigmaHdg: 15,
            sigmaM: 1000,
            timestamp: Date.now(),
            source: 'gps_heading',
            weight: 0.7,
          });
        }

        ekfRef.current.updateSpeed({
          type: 'obd',
          speed: SIMULATION_SPEED,
          sigmaM: 0.16,
          timestamp: Date.now(),
          source: 'obd_sim',
          weight: 0.95,
        });

        const currentPose = ekfRef.current.getPose();
        const matched = matcherRef.current.match(currentPose);
        const state = guidanceRef.current.update(currentPose, matched);
        
        const accelZ = 9.81 + (Math.random() - 0.5) * 0.5;
        const detected = anomalyRef.current.ingest({
          ax: (Math.random() - 0.5) * 0.3,
          ay: (Math.random() - 0.5) * 0.3,
          az: accelZ,
          timestamp: Date.now(),
          speed: SIMULATION_SPEED,
          position: currentPose.position,
          matchedSegment: matched,
        });

        setPose(currentPose);
        setNavState(state);
        
        if (detected) {
          setAnomalies(prev => [...prev, detected]);
          if (detected.severity > 0.6) {
            setShowAnomalyAlert(detected);
            setTimeout(() => setShowAnomalyAlert(null), 3000);
          }
        }

        const distPerTick = SIMULATION_SPEED * (TICK_INTERVAL / 1000);
        const segmentDist = haversine(wp, newRoute.waypoints[Math.min(idx + 1, newRoute.waypoints.length - 1)]);
        simIndexRef.current += distPerTick / Math.max(segmentDist, 1);
        if (simIndexRef.current >= newRoute.waypoints.length - 1) {
          simIndexRef.current = newRoute.waypoints.length - 1;
          setScreen('arrived');
          return;
        }
      }

      navTickRef.current = window.setTimeout(tick, TICK_INTERVAL);
    };

    tick();
  }, [shieldMode, pose.heading]);

  const stopNavigation = useCallback(() => {
    if (navTickRef.current) {
      clearTimeout(navTickRef.current);
      navTickRef.current = null;
    }
    setScreen('map');
    setRoute(null);
    setNavState(null);
    guidanceRef.current.setRoute(null);
    matcherRef.current.reset();
    simIndexRef.current = 0;
  }, []);

  useEffect(() => {
    return () => {
      if (navTickRef.current) clearTimeout(navTickRef.current);
    };
  }, []);

  const handleSearch = (poi: POI) => {
    setSelectedPOI(poi);
    setSearchOpen(false);
    setPose(prev => ({
      ...prev,
      position: { lat: poi.lat, lng: poi.lng },
    }));
  };

  const quickStart = () => {
    const randomPOI = dubaiPOIs[Math.floor(Math.random() * dubaiPOIs.length)];
    startNavigation(randomPOI);
  };

  return (
    <div className="relative w-full h-screen overflow-hidden bg-black">
      <div className="absolute inset-0 z-0">
        <Map
          pose={pose}
          route={route}
          destination={selectedPOI}
          anomalies={anomalies}
          mapStyle={mapStyle}
          shieldMode={shieldMode}
          isNavigating={screen === 'navigating'}
        />
      </div>

      <div className="absolute top-0 left-0 right-0 z-20 p-4">
        <div className="flex items-center justify-between">
          <button 
            onClick={() => setMenuOpen(!menuOpen)}
            className="hud-panel w-10 h-10 rounded-xl flex items-center justify-center active:scale-95 transition-transform"
          >
            <Menu className="w-5 h-5" />
          </button>

          <button
            onClick={() => setShieldMode(!shieldMode)}
            className={`hud-panel px-3 py-2 rounded-xl flex items-center gap-2 text-sm font-semibold transition-all ${
              shieldMode ? 'bg-orange-500 text-white border-orange-400' : ''
            }`}
          >
            <Shield className="w-4 h-4" />
            <span>{shieldMode ? 'Shield ON' : 'Shield OFF'}</span>
          </button>

          <button
            onClick={() => setMapStyle(s => s === 'dark' ? 'light' : s === 'light' ? 'satellite' : 'dark')}
            className="hud-panel w-10 h-10 rounded-xl flex items-center justify-center text-xs font-bold"
          >
            {mapStyle === 'dark' ? 'D' : mapStyle === 'light' ? 'L' : 'S'}
          </button>
        </div>
      </div>

      {menuOpen && (
        <div className="absolute top-16 left-4 z-30 hud-panel rounded-2xl p-4 w-56 space-y-2">
          <h3 className="font-bold text-lg mb-3">CAMEL Navigation</h3>
          <button 
            onClick={() => { setSearchOpen(true); setMenuOpen(false); }}
            className="w-full text-left px-3 py-2 rounded-lg hover:bg-orange-50 transition-colors flex items-center gap-2"
          >
            <Navigation className="w-4 h-4" /> Navigate
          </button>
          <button 
            onClick={() => { setMenuOpen(false); }}
            className="w-full text-left px-3 py-2 rounded-lg hover:bg-orange-50 transition-colors flex items-center gap-2"
          >
            <Locate className="w-4 h-4" /> Recenter
          </button>
          <div className="border-t pt-2 mt-2">
            <p className="text-xs text-muted-foreground px-3">v2.0.0 Production</p>
            <p className="text-xs text-muted-foreground px-3">Offline Dubai Ready</p>
          </div>
        </div>
      )}

      {searchOpen && (
        <SearchPanel
          onClose={() => setSearchOpen(false)}
          onSelectPOI={handleSearch}
          onStartNavigation={startNavigation}
          currentPosition={pose.position}
        />
      )}

      {screen === 'navigating' && navState && (
        <>
          <div className="absolute top-20 left-4 right-4 z-20">
            <ManeuverCard navState={navState} />
          </div>

          <div className="absolute bottom-32 left-4 right-4 z-20">
            <DriveHUD navState={navState} route={route} />
          </div>

          <div className="absolute bottom-36 right-4 z-20">
            <Speedometer speed={pose.speed} speedLimit={getSpeedLimit(navState)} />
          </div>

          <button
            onClick={stopNavigation}
            className="absolute bottom-24 left-1/2 -translate-x-1/2 z-20 bg-red-500 hover:bg-red-600 text-white px-6 py-3 rounded-2xl font-bold shadow-lg flex items-center gap-2 active:scale-95 transition-all"
          >
            <X className="w-5 h-5" />
            End Navigation
          </button>

          {showAnomalyAlert && (
            <div className="absolute top-36 left-4 right-4 z-30 anomaly-alert">
              <div className="bg-red-500 text-white px-4 py-3 rounded-2xl shadow-lg flex items-center gap-3">
                <AlertTriangle className="w-6 h-6 flex-shrink-0" />
                <div>
                  <p className="font-bold text-sm">Rough Road Ahead!</p>
                  <p className="text-xs opacity-90">{showAnomalyAlert.type.replace('_', ' ')} detected</p>
                </div>
              </div>
            </div>
          )}
        </>
      )}

      {screen === 'arrived' && (
        <div className="absolute inset-0 z-30 bg-black/60 flex items-center justify-center backdrop-blur-sm">
          <div className="hud-panel rounded-3xl p-8 max-w-sm w-full mx-4 text-center space-y-4">
            <div className="w-20 h-20 bg-gradient-to-br from-orange-400 to-orange-600 rounded-full flex items-center justify-center mx-auto shadow-lg">
              <Navigation className="w-10 h-10 text-white" />
            </div>
            <h2 className="text-2xl font-bold">You Have Arrived!</h2>
            <p className="text-muted-foreground">{selectedPOI?.name}</p>
            <p className="text-sm text-muted-foreground">{selectedPOI?.address}</p>
            <button
              onClick={stopNavigation}
              className="w-full bg-orange-500 hover:bg-orange-600 text-white py-3 rounded-2xl font-bold transition-colors"
            >
              Done
            </button>
          </div>
        </div>
      )}

      {screen === 'map' && (
        <BottomBar
          onSearch={() => setSearchOpen(true)}
          onQuickStart={quickStart}
          shieldMode={shieldMode}
          pose={pose}
        />
      )}

      {selectedPOI && screen === 'map' && !searchOpen && (
        <div className="absolute bottom-28 left-4 right-4 z-20">
          <div className="hud-panel rounded-2xl p-4 flex items-center gap-4">
            <div className="w-12 h-12 bg-orange-100 rounded-xl flex items-center justify-center text-2xl flex-shrink-0">
              {selectedPOI.category === 'restaurant' ? '🍽️' :
               selectedPOI.category === 'shopping' ? '🛍️' :
               selectedPOI.category === 'hotel' ? '🏨' :
               selectedPOI.category === 'landmark' ? '🏛️' : '📍'}
            </div>
            <div className="flex-1 min-w-0">
              <h3 className="font-bold truncate">{selectedPOI.name}</h3>
              <p className="text-xs text-muted-foreground truncate">{selectedPOI.address}</p>
            </div>
            <button
              onClick={() => startNavigation(selectedPOI)}
              className="bg-orange-500 hover:bg-orange-600 text-white px-4 py-2 rounded-xl font-semibold text-sm transition-colors flex-shrink-0"
            >
              Go
            </button>
          </div>
        </div>
      )}
    </div>
  );
}

function haversine(a: GeoPoint, b: GeoPoint): number {
  const R = 6371000;
  const dLat = (b.lat - a.lat) * Math.PI / 180;
  const dLng = (b.lng - a.lng) * Math.PI / 180;
  const c = 2 * Math.asin(Math.sqrt(
    Math.sin(dLat / 2) ** 2 + Math.cos(a.lat * Math.PI / 180) * Math.cos(b.lat * Math.PI / 180) * Math.sin(dLng / 2) ** 2
  ));
  return R * c;
}

function getSpeedLimit(navState: NavigationState): number {
  const edge = navState.route?.edges[0];
  return edge?.speedLimit || 80;
}
