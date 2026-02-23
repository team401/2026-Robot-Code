export interface Rotation3d {
  m_q: Quaternion;
}

export interface Quaternion {
  m_w: number;
  m_x: number;
  m_y: number;
  m_z: number;
}

export interface JSONAPTarget {
  kind: "JSONAPTarget";
  reference: Pose2d;
  entryAngle: Rotation2d;
  velocity: number;
  rotationRadius: Distance;
}

export interface LowLevelControlElement {
  elementType: string;
  stringID: string;
  inverted: boolean;
  minValue: number;
  maxValue: number;
  clampValue: boolean;
}

export interface JSONTranslation2d {
  kind: "JSONTranslation2d";
  x: number;
  y: number;
}

export interface JSONPer {
  kind: "JSONPer";
  dividend: Measure;
  divisor: Measure;
}

export interface JSONTranslation3d {
  kind: "JSONTranslation3d";
  x: number;
  y: number;
  z: number;
}

export interface JSONTransform2d {
  kind: "JSONTransform2d";
  rotation: Rotation2d;
  translation: Translation2d;
}

export interface JSONTransform3d {
  kind: "JSONTransform3d";
  rotation: Rotation3d;
  translation: Translation3d;
}

export interface ControllerJsonRepresentation {
  kind: "ControllerJsonRepresentation";
  port: number;
  type: string;
  controlElements: ControlElement[];
  buttonShorthands: { [key: string]: number };
  axisShorthands: { [key: string]: number };
  povShorthands: { [key: string]: number };
}

export interface Translation3d {
  m_x: number;
  m_y: number;
  m_z: number;
}

export interface JSONRotation2d {
  kind: "JSONRotation2d";
  radians: number;
}

export interface JSONPose2d {
  kind: "JSONPose2d";
  rotation: Rotation2d;
  translation: Translation2d;
}

export interface Rotation2d {
  m_value: number;
  m_cos: number;
  m_sin: number;
}

export interface JSONPose3d {
  kind: "JSONPose3d";
  rotation: Rotation3d;
  translation: Translation3d;
}

export interface Translation2d {
  m_x: number;
  m_y: number;
}

export interface Distance {
}

export interface Pose2d {
  m_translation: Translation2d;
  m_rotation: Rotation2d;
}

export interface Measure {
}

export interface JSONRotation3d {
  kind: "JSONRotation3d";
  roll: number;
  pitch: number;
  yaw: number;
}

export interface ControlElement {
  lowLevelControlElement: LowLevelControlElement;
  command: string;
  commandType: string;
  inverted: boolean;
  minValue: number;
  maxValue: number;
  clampValue: boolean;
}

export type JSONObject =
  | JSONRotation3d
  | JSONRotation2d
  | JSONPer
  | JSONTranslation2d
  | JSONPose2d
  | JSONAPTarget
  | JSONPose3d
  | ControllerJsonRepresentation
  | JSONTransform3d
  | JSONTranslation3d
  | JSONTransform2d;

