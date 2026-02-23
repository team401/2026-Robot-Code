export interface RoutineCall {
  type: "RoutineCall";
  routineName: string;
}

export interface Deadline {
  type: "Deadline";
  deadline: AutoAction;
  others: AutoAction[];
}

export interface Parallel {
  type: "Parallel";
  actions: AutoAction[];
}

export interface Race {
  type: "Race";
  actions: AutoAction[];
}

export interface JSONRotation2d {
  radians: number;
}

export interface JSONTranslation2d {
  x: number;
  y: number;
}

export interface JSONPose2d {
  rotation: JSONRotation2d;
  translation: JSONTranslation2d;
}

export interface JSONMeasure {
  value: number;
  unit: string;
}

export interface JSONAPTarget {
  reference: JSONPose2d;
  entryAngle: JSONRotation2d;
  velocity: number;
  rotationRadius: JSONMeasure;
}

export interface APConstraints {
  velocity: number;
  acceleration: number;
  jerk: number;
}

export interface APProfile {
  constraints: APConstraints;
  errorXY: JSONMeasure;
  errorTheta: JSONMeasure;
  beelineRadius: JSONMeasure;
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

export interface AutoPilotAction {
  type: "AutoPilotAction";
  target: JSONAPTarget;
  profile: APProfile;
  constraints: APConstraints;
  pidGains: PIDGains;
}

export type AutoAction = RoutineCall | Deadline | Sequence | Parallel | Race | AutoPilotAction;

export interface Sequence {
  actions: AutoAction[];
}

