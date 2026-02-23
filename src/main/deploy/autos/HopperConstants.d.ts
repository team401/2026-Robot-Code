export type InvertedValue = "CounterClockwise_Positive" | "Clockwise_Positive";

export interface HoodConstants {
  disconnectedDebounceTimeSeconds: number;
  mechanismAngleToExitAngle: JSONMeasure;
  hoodKP: number;
  hoodKI: number;
  hoodKD: number;
  hoodKS: number;
  hoodKG: number;
  hoodKV: number;
  hoodKA: number;
  hoodExpoKV: number;
  hoodExpoKA: number;
  hoodReduction: number;
  hoodSupplyCurrentLimit: JSONMeasure;
  hoodStatorCurrentLimit: JSONMeasure;
  hoodMotorDirection: InvertedValue;
  minHoodAngle: JSONMeasure;
  maxHoodAngle: JSONMeasure;
  homingVoltage: JSONMeasure;
  homingMovementThreshold: JSONMeasure;
  homingMaxUnmovingTime: JSONMeasure;
  simHoodMOI: JSONMeasure;
  simHoodArmLength: JSONMeasure;
}

