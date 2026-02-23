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

