export interface GainConstants {
  maxAcceptedDistanceMeters: number;
  linearStdDevFactor: number;
  angularStdDevFactor: number;
  maxZCutoff: number;
  maxSingleTagAmbiguity: number;
  maxAmbiguity: number;
}

export interface Rotation {
  roll: number;
  pitch: number;
  yaw: number;
}

export interface Translation {
  x: number;
  y: number;
  z: number;
}

export interface CameraTransform {
  rotation: Rotation;
  translation: Translation;
}

export interface CameraConfig {
  index: number;
  name: string;
  transform: CameraTransform;
}

export interface VisionConfig {
  gainConstants: GainConstants;
  cameras: CameraConfig[];
}

export interface VisionWireFormat {
  gainConstants: GainConstants;
  [key: string]: unknown;
}
