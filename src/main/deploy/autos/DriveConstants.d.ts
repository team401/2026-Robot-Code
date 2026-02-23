export interface JSONMeasure {
  value: number;
  unit: string;
}

export interface PIDGains {
  kP: number;
  kI: number;
  kD: number;
  kS: number;
  kG: number;
  kV: number;
  kA: number;
}

export interface DriveConstants {
  maxLinearSpeed: JSONMeasure;
  maxAngularSpeed: JSONMeasure;
  joystickDeadband: number;
  joystickMagnitudeExponent: number;
  linearDriveProfileMaxLinearVelocity: JSONMeasure;
  linearDriveProfileMaxAngularVelocity: JSONMeasure;
  linearDriveProfileMaxLinearAcceleration: JSONMeasure;
  linearDriveProfileMaxAngularAcceleration: JSONMeasure;
  linearDriveMaxAngularError: JSONMeasure;
  linearDriveMaxPositionError: JSONMeasure;
  linearDriveMaxLinearVelocityError: JSONMeasure;
  linearDriveMaxAngularVelocityError: JSONMeasure;
  steerGains: PIDGains;
  driveGains: PIDGains;
  defaultAutoPilotHeadingGains: PIDGains;
  defaultAutoPilotBeelineRadius: JSONMeasure;
  defaultAutoPilotHeadingTolerance: JSONMeasure;
  defaultAutoPilotXYTolerance: JSONMeasure;
}

