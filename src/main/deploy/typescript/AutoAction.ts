export class Deadline {
  type: "Deadline" = "Deadline";
  deadline?: AutoAction;
  others?: AutoAction[];
	constructor({deadline = undefined, others = []}: Partial<{deadline: AutoAction; others: AutoAction[]}>) {
    this.deadline = deadline;
    this.others = others;
  }
}

export class Sequence {
  type: "Sequence" = "Sequence";
  actions?: AutoAction[];
	constructor({actions = []}: Partial<{actions: AutoAction[]}>) {
    this.actions = actions;
  }
}

export class Parallel {
  type: "Parallel" = "Parallel";
  actions?: AutoAction[];
	constructor({actions = []}: Partial<{actions: AutoAction[]}>) {
    this.actions = actions;
  }
}

export class Race {
  type: "Race" = "Race";
  actions?: AutoAction[];
	constructor({actions = []}: Partial<{actions: AutoAction[]}>) {
    this.actions = actions;
  }
}

export class Rotation2d {
  radians?: number;
	constructor({radians = 0}: Partial<{radians: number}>) {
    this.radians = radians;
  }
}

export class Translation2d {
  x?: number;
  y?: number;
	constructor({x = 0, y = 0}: Partial<{x: number; y: number}>) {
    this.x = x;
    this.y = y;
  }
}

export class Pose2d {
  rotation?: Rotation2d;
  translation?: Translation2d;
	constructor({rotation = new Rotation2d({}), translation = new Translation2d({})}: Partial<{rotation: Rotation2d; translation: Translation2d}>) {
    this.rotation = rotation;
    this.translation = translation;
  }
}

import * as Units from './Units.js';

export class APTarget {
  reference?: Pose2d;
  entryAngle?: Rotation2d;
  velocity?: number;
  rotationRadius?: Units.Distance;
	constructor({reference = new Pose2d({}), entryAngle = new Rotation2d({}), velocity = 0, rotationRadius = undefined}: Partial<{reference: Pose2d; entryAngle: Rotation2d; velocity: number; rotationRadius: Units.Distance}>) {
    this.reference = reference;
    this.entryAngle = entryAngle;
    this.velocity = velocity;
    this.rotationRadius = rotationRadius;
  }
}

export class APConstraints {
  velocity?: number;
  acceleration?: number;
  jerk?: number;
	constructor({velocity = 0, acceleration = 0, jerk = 0}: Partial<{velocity: number; acceleration: number; jerk: number}>) {
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.jerk = jerk;
  }
}

export class APProfile {
  constraints?: APConstraints;
  errorXY?: Units.Distance;
  errorTheta?: Units.Angle;
  beelineRadius?: Units.Distance;
	constructor({constraints = new APConstraints({}), errorXY = undefined, errorTheta = undefined, beelineRadius = undefined}: Partial<{constraints: APConstraints; errorXY: Units.Distance; errorTheta: Units.Angle; beelineRadius: Units.Distance}>) {
    this.constraints = constraints;
    this.errorXY = errorXY;
    this.errorTheta = errorTheta;
    this.beelineRadius = beelineRadius;
  }
}

export class PIDGains {
  kP?: number;
  kI?: number;
  kD?: number;
  kS?: number;
  kG?: number;
  kV?: number;
  kA?: number;
	constructor({kP = 0, kI = 0, kD = 0, kS = 0, kG = 0, kV = 0, kA = 0}: Partial<{kP: number; kI: number; kD: number; kS: number; kG: number; kV: number; kA: number}>) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kG = kG;
    this.kV = kV;
    this.kA = kA;
  }
}

export class AutoPilotAction {
  type: "AutoPilotAction" = "AutoPilotAction";
  target?: APTarget;
  profile?: APProfile;
  constraints?: APConstraints;
  pidGains?: PIDGains;
	constructor({target = new APTarget({}), profile = new APProfile({}), constraints = new APConstraints({}), pidGains = new PIDGains({})}: Partial<{target: APTarget; profile: APProfile; constraints: APConstraints; pidGains: PIDGains}>) {
    this.target = target;
    this.profile = profile;
    this.constraints = constraints;
    this.pidGains = pidGains;
  }
}

export type AutoAction = Deadline | Sequence | Parallel | Race | AutoPilotAction| undefined | null;

