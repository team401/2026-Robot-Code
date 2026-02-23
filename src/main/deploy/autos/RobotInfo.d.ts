export interface JSONMeasure {
  value: number;
  unit: string;
}

export interface JSONRotation3d {
  roll: number;
  pitch: number;
  yaw: number;
}

export interface JSONTranslation3d {
  x: number;
  y: number;
  z: number;
}

export interface JSONTransform3d {
  rotation: JSONRotation3d;
  translation: JSONTranslation3d;
}

export type CANdiSignal = "S1" | "S2";

export interface RobotInfo {
  canivoreBusName: string;
  logFilePath: string;
  robotMass: JSONMeasure;
  robotMOI: JSONMeasure;
  wheelCof: number;
  robotPeriod: JSONMeasure;
  robotToShooter: JSONTransform3d;
  homingSwitchSignal: CANdiSignal;
}

