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

export interface AutoPilotAction {
  type: "AutoPilotAction";
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

export interface JSONAPTarget {
  reference: JSONPose2d;
  entryAngle: JSONRotation2d;
  velocity: number;
  rotationRadius: JSONMeasure;
}

  target: JSONAPTarget;
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

  profile: APProfile;
  constraints: APConstraints;
  pidGains: PIDGains;
}

export type AutoAction = RoutineCall | Deadline | Sequence | Parallel | Race | AutoPilotAction;

export interface Sequence {
  actions: AutoAction[];
}

