export interface TuningAttempt {
  id: string;
  createdAt: string;
  /** Distance computed from robot pose vs hub center (meters) */
  distanceMeters: number;
  /** Distance reported directly by the robot via NT (meters); null for old attempts */
  distanceToHubMeters: number | null;
  /** Shooter setpoint in RPM (from TunableNumbers) */
  shooterRPM: number;
  /** Hood setpoint in degrees (from TunableNumbers) */
  hoodAngleDegrees: number;
  robotPoseX: number;
  robotPoseY: number;
  leavesShooterTimeSec: number | null;
  hitTargetTimeSec: number | null;
  flightTimeSec: number | null;
  exportedToShotMap: 'hub' | 'pass' | null;
  fps: number;
}

// 
// coordinates below were obtained via FieldConstants.Hub.oppInnerCenterPoint(); 
// (red hub, blue origin)
// - gback
export const HUB_CENTER = { 
    x: 11.894744,
    y: 4.021500
} as const;
