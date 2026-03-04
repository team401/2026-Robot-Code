# Shot Map Tuning Tab - Implementation Plan

## Context

The FRC robot (Team 401) has a shooting mechanism for launching balls into a "hub" target. To tune shot parameters, we need an in-shop workflow that captures video of each shot alongside telemetry (distance, RPM, hood angle), then lets the user analyze the video frame-by-frame to determine flight time. This data feeds into the existing Shot Maps interpolation tables.

The settings_gui app (React 19 / MUI 7 / Vite 7 / TypeScript) currently has Shot Maps and Vision tabs. This plan adds a "Shot Map Tuning" tab with two new capabilities: **NT4 real-time telemetry** and **browser-based video recording/replay**.

---

## New Files

```
settings_gui/
  src/
    types/ShotTuning.ts                      # Data model
    services/nt4.ts                          # NT4 WebSocket client wrapper
    services/shotTuningStorage.ts            # Persistence API (fetch to Vite middleware)
    pages/ShotMapTuning.tsx                  # Main page component
    components/shot-tuning/
      NTConnectionStatus.tsx                 # Connection target selector + status
      TelemetryDisplay.tsx                   # Live distance/RPM/hood angle readout
      RecordingControls.tsx                  # Start/Stop/Discard/Store + live camera
      AttemptsList.tsx                       # List stored attempts, select, delete
      VideoReplayPlayer.tsx                  # Playback, frame-step, mark timestamps
  shot-tuning-data/                          # Created at runtime
    attempts.json                            # Attempt metadata
    clips/<id>.webm                          # Video files
```

## Modified Files

- **`package.json`** -- add `ntcore-ts-client` dependency
- **`vite.config.ts`** -- add `shotTuningPlugin()` middleware for JSON + binary persistence
- **`src/endpoints/registry.ts`** -- register new tab

---

## Data Model (`src/types/ShotTuning.ts`)

```typescript
export interface TuningAttempt {
  id: string;                        // UUID, doubles as .webm filename
  createdAt: string;                 // ISO 8601
  distanceMeters: number;            // 2D distance to hub at capture time
  shooterRPMRadPerSec: number;       // Avg of lead + follower velocity
  hoodAngleRadians: number;          // Hood position
  robotPoseX: number;               // For reference
  robotPoseY: number;
  leavesShooterTimeSec: number | null;
  hitTargetTimeSec: number | null;
  flightTimeSec: number | null;      // Computed difference
  exportedToShotMap: 'hub' | 'pass' | null;
}
export const HUB_CENTER = { x: 4.02844, y: 4.00050 } as const;
```

## NT4 Service (`src/services/nt4.ts`)

- Uses `ntcore-ts-client` (NT4 WebSocket protocol, port 5810)
- Targets: `localhost` (simulation) or `10.4.1.2` (robot)
- Subscriptions:
  - **Odometry/Robot** -- Pose2d struct (24 bytes: 3x float64 LE) -> parse x, y -> compute `sqrt((x-hubX)^2 + (y-hubY)^2)`
  - **Shooter/LeadMotorInputs/velocityRadiansPerSecond** + **Shooter/FollowerMotorInputs/velocityRadiansPerSecond** -- average both doubles
  - **Hood/inputs/positionRadians** -- single double
- Fallback if struct parsing fails: add `Logger.recordOutput` for x/y individually in robot code

## Persistence (`vite.config.ts` + `shotTuningStorage.ts`)

New Vite middleware routes (following existing `localFilesPlugin` pattern):
- `GET /shot-tuning/attempts` -- read `attempts.json`
- `POST /shot-tuning/attempts` -- write `attempts.json`
- `POST /shot-tuning/clips/:id` -- save binary .webm
- `GET /shot-tuning/clips/:id` -- serve .webm for playback
- `DELETE /shot-tuning/clips/:id` -- remove clip file

## Video Recording

- `navigator.mediaDevices.getUserMedia({ video: true })` for webcam
- `MediaRecorder` with `video/webm` (detect best codec: vp9 > vp8)
- On **Start**: snapshot telemetry, begin recording, show live feed
- On **Stop**: assemble blob, show Discard/Store buttons
- **Store**: upload clip, create attempt record, save metadata

## Video Replay

- `<video>` element with `/shot-tuning/clips/<id>` as src
- Play/Pause, current time display
- Frame-step: +/- 1/30s via `video.currentTime` adjustment
- "Mark Leaves Shooter" / "Mark Hit Target" buttons save `currentTime`
- Flight time auto-computed when both marks set

## Shot Maps Integration

Per-attempt buttons: "Add to Hub Shot Map" / "Add to Pass Shot Map"
- Convert: rad/s -> RPM, radians -> degrees, meters stays
- Load `ShotMaps.json` via existing `loadLocal()`, append data point, save via `saveLocal()`
- Mark attempt as exported

---

## Implementation Phases

### Phase 1: Foundation
1. `npm i ntcore-ts-client`
2. Create `src/types/ShotTuning.ts`
3. Add `shotTuningPlugin()` to `vite.config.ts`
4. Create `src/services/shotTuningStorage.ts`
5. Create skeleton `src/pages/ShotMapTuning.tsx`
6. Register in `src/endpoints/registry.ts`

### Phase 2: Video Recording
7. Build `RecordingControls.tsx` with webcam + MediaRecorder
8. Wire into main page, test recording/discard/store flow

### Phase 3: NT4 Telemetry
9. Build `src/services/nt4.ts`
10. Build `NTConnectionStatus.tsx` and `TelemetryDisplay.tsx`
11. Wire telemetry snapshot into recording flow

### Phase 4: Replay & Analysis
12. Build `VideoReplayPlayer.tsx` with frame-stepping and timestamp marking
13. Build `AttemptsList.tsx` with select/delete
14. Wire flight time computation

### Phase 5: Shot Maps Export
15. Add export buttons, conversion logic, integration with existing `loadLocal`/`saveLocal`

---

## Verification

1. `npm run dev` -- app starts, new tab appears in sidebar
2. Click "Shot Map Tuning" -- page loads with NT connection controls
3. Grant camera permission -- live feed visible
4. Start/Stop recording -- can discard or store
5. Stored attempt appears in list, video replays
6. Frame-step and mark timestamps -> flight time computes
7. Connect to simulation (`localhost`) -- telemetry values update live
8. Export to shot map -> values appear in Shot Maps tab
