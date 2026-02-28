export interface TuningAttempt {
  id: string;
  createdAt: string;
  distanceMeters: number;
  shooterRPMRadPerSec: number;
  hoodAngleRadians: number;
  robotPoseX: number;
  robotPoseY: number;
  leavesShooterTimeSec: number | null;
  hitTargetTimeSec: number | null;
  flightTimeSec: number | null;
  exportedToShotMap: 'hub' | 'pass' | null;
  fps: number;
}

// 
// coordinates below were obtained via FieldConstants.Hub.topCenterPoint();
// - gback
export const HUB_CENTER = { 
    x: 4.604766,
    y: 4.021500,
} as const;
