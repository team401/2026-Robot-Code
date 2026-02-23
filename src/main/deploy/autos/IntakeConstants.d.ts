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

export interface IntakeConstants {
  pivotReduction: number;
  rollersReduction: number;
  armLength: JSONMeasure;
  minPivotAngle: JSONMeasure;
  maxPivotAngle: JSONMeasure;
  pivotStartingAngle: JSONMeasure;
  rollersInertia: JSONMeasure;
  pivotInertia: JSONMeasure;
  intakePositionAngle: JSONMeasure;
  stowPositionAngle: JSONMeasure;
  intakeRollerSpeed: JSONMeasure;
  homingMovementThreshold: JSONMeasure;
  homingTimeoutSeconds: JSONMeasure;
  homingVoltage: JSONMeasure;
  pivotPIDGains: PIDGains;
  rollersPIDGains: PIDGains;
  pivotSupplyCurrentLimit: JSONMeasure;
  pivotStatorCurrentLimit: JSONMeasure;
  rollersStatorCurrentLimit: JSONMeasure;
  rollersSupplyCurrentLimit: JSONMeasure;
}

