export interface Rotation3d {
  roll: number;
  pitch: number;
  yaw: number;
}

export interface Rotation2d {
  radians: number;
}

export interface Per {
  dividend: Measure;
  divisor: Measure;
}

export interface Translation2d {
  x: number;
  y: number;
}

export interface Pose2d {
  rotation: Rotation2d;
  translation: Translation2d;
}

export interface APTarget {
  reference: Pose2d;
  entryAngle: Rotation2d;
  velocity: number;
  rotationRadius: Distance;
}

export interface Pose3d {
  rotation: Rotation3d;
  translation: Translation3d;
}

export interface Controller {
  port: number;
  type: string;
  controlElements: ControlElement[];
  buttonShorthands: { [key: string]: number };
  axisShorthands: { [key: string]: number };
  povShorthands: { [key: string]: number };
}

export interface Transform3d {
  rotation: Rotation3d;
  translation: Translation3d;
}

export interface Translation3d {
  x: number;
  y: number;
  z: number;
}

export interface Transform2d {
  rotation: Rotation2d;
  translation: Translation2d;
}

