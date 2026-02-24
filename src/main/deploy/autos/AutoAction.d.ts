export interface RoutineCall {
    type: "RoutineCall";
    routineName: string;
}
export interface Deadline {
    type: "Deadline";
    deadline: AutoAction;
    others: AutoAction[];
}
export interface Sequence {
    type: "Sequence";
    actions: AutoAction[];
}
export interface Parallel {
    type: "Parallel";
    actions: AutoAction[];
}
export interface Race {
    type: "Race";
    actions: AutoAction[];
}
export interface Rotation2d {
    radians: number;
}
export interface Translation2d {
    x: number;
    y: number;
}
export interface Pose2d {
    rotation: Rotation2d;
    translation: Translation2d;
}
import * as Units from './Units';
export interface APTarget {
    reference: Pose2d;
    entryAngle: Rotation2d;
    velocity: number;
    rotationRadius: Units.Distance;
}
export interface APConstraints {
    velocity: number;
    acceleration: number;
    jerk: number;
}
export interface APProfile {
    constraints: APConstraints;
    errorXY: Units.Distance;
    errorTheta: Units.Angle;
    beelineRadius: Units.Distance;
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
    target: APTarget;
    profile: APProfile;
    constraints: APConstraints;
    pidGains: PIDGains;
}
export type AutoAction = RoutineCall | Deadline | Sequence | Parallel | Race | AutoPilotAction;
//# sourceMappingURL=AutoAction.d.ts.map