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

export const HUB_CENTER = { x: 4.02844, y: 4.00050 } as const;
