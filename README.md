# CAMEL Navigation

**Production-grade, Dubai-first offline navigation system.**

> Android-focused. Classical sensor fusion. 3D maps. No NPU required.

## Live Demo

**[Try it now](https://egymoqk5a2xgw.kimi.page)**

## What Makes This Different

| Feature | CAMEL | Google Maps | Waze |
|---------|-------|-------------|------|
| Offline from first launch | Yes | Partial | No |
| WiFi multilateration | Yes | No | No |
| EKF sensor fusion | Yes | Proprietary | Proprietary |
| Road anomaly detection | Yes | No | No |
| Shield mode (GPS-free) | Yes | No | No |
| Road signature learning | Yes | No | No |
| Open architecture | Yes | No | No |

## Architecture

```
User → React UI → Navigation Engine → Mapbox GL 3D
                     ↓
              EKF (6-DOF state)
                     ↓
    ┌────────┬──────────┬──────────┬─────────┐
    GPS    WiFi+RANSAC  Cell      IMU+ZUPT
```

## Tech Stack

- **Frontend**: React 19 + TypeScript + Tailwind CSS + shadcn/ui
- **Maps**: Mapbox GL JS with 3D buildings + terrain
- **Engine**: Pure TypeScript, zero dependencies
- **Navigation**: EKF + HMM map matcher + A* router
- **Sensors**: GPS + WiFi multilateration + cell tower + IMU fusion

## Key Components

### Navigation Engine (`src/engine/`)

| Module | Purpose |
|--------|---------|
| `ekf.ts` | Extended Kalman Filter - 6-DOF state estimation |
| `router.ts` | A* routing on Dubai road graph |
| `matcher.ts` | HMM map matching - road-snapped positioning |
| `guidance.ts` | Turn-by-turn instructions + ETA |
| `sensors.ts` | GPS, WiFi RANSAC, cell, IMU adapters |
| `anomalies.ts` | Pothole/speed bump detection from accelerometer |

### Data (`src/data/`)

| File | Contents |
|------|----------|
| `dubaiRoads.ts` | 81-node Dubai road graph (pre-bundled) |
| `dubaiPOIs.ts` | 80+ points of interest across Dubai |

### UI (`src/components/`)

| Component | Purpose |
|-----------|---------|
| `Map.tsx` | 3D Mapbox map with route + anomaly overlays |
| `SearchPanel.tsx` | POI search with LRU cache + debounce |
| `ManeuverCard.tsx` | Turn-by-turn instruction card |
| `DriveHUD.tsx` | Navigation status + lane guidance |
| `Speedometer.tsx` | Canvas speedometer with speed limit |
| `BottomBar.tsx` | Main action bar |

## Running Locally

```bash
npm install
npm run build
npm run preview
```

## Dubai Road Network Coverage

- Sheikh Zayed Road (full corridor)
- Al Khail Road (eastern bypass)
- Emirates Road E611 (outer ring)
- Al Ain Road (east-west)
- Airport corridor
- Deira / Old Dubai
- Marina / JBR corridor
- Downtown connectors
- JLT / Media City / Internet City

## POI Categories

- 10+ major landmarks (Burj Khalifa, Dubai Mall, Atlantis, etc.)
- 10+ shopping malls
- 10+ hotels
- 8+ restaurants
- 5+ hospitals
- 7+ business districts
- 8+ parks & beaches
- 10+ metro/tram stations
- 3+ mosques

## License

MIT
