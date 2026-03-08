export type DistanceSource = 'networkTables' | 'odometry';

export interface TargetCoordinates {
  x: number;
  y: number;
}

export interface TuningAttempt {
  id: string;
  createdAt: string;
  /** Distance computed from robot pose vs the configured target (meters) */
  distanceMeters: number;
  /** Distance reported directly by the robot via NT (meters); retained field name for compatibility */
  distanceToHubMeters: number | null;
  /** Distance recorded for this sample based on the selected source at capture time */
  sampleDistanceMeters?: number;
  /** Source selected when this sample was captured */
  distanceSource?: DistanceSource;
  /** Target coordinates used when computing the odometry distance */
  targetCoordinates?: TargetCoordinates;
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

// Coordinates below were obtained via FieldConstants.Hub.oppInnerCenterPoint().
// This remains the default target so existing hub tuning behavior is unchanged.
export const DEFAULT_TARGET_COORDINATES = {
  x: 11.894744,
  y: 4.021500,
} as const satisfies TargetCoordinates;

export const DEFAULT_DISTANCE_SOURCE: DistanceSource = 'networkTables';

export function getAttemptDistanceSource(attempt: TuningAttempt): DistanceSource {
  if (attempt.distanceSource) return attempt.distanceSource;
  return attempt.distanceToHubMeters !== null ? 'networkTables' : 'odometry';
}

export function getAttemptTargetCoordinates(attempt: TuningAttempt): TargetCoordinates {
  return attempt.targetCoordinates ?? DEFAULT_TARGET_COORDINATES;
}

export function getAttemptSampleDistance(attempt: TuningAttempt): number {
  if (attempt.sampleDistanceMeters !== undefined) return attempt.sampleDistanceMeters;
  return getAttemptDistanceSource(attempt) === 'networkTables'
    ? attempt.distanceToHubMeters ?? attempt.distanceMeters
    : attempt.distanceMeters;
}
