import * as Units from './Units';

export interface Rotation3d {
  roll: number;
  pitch: number;
  yaw: number;
}

export interface Translation3d {
  x: number;
  y: number;
  z: number;
}

export interface Transform3d {
  rotation: Rotation3d;
  translation: Translation3d;
}

export type CANdiSignal = "S1" | "S2";

export interface RobotInfo {
  canivoreBusName: string;
  logFilePath: string;
  robotMass: Units.Mass;
  robotMOI: Units.MomentOfInertia;
  wheelCof: number;
  robotPeriod: Units.Time;
  robotToShooter: Transform3d;
  homingSwitchSignal: CANdiSignal;
}

export type FieldType = "ANDYMARK" | "WELDED";

export interface AprilTagConstants {
  fieldType: FieldType;
}

export interface CANBusAssignment {
  frontLeftDriveKrakenId: number;
  frontLeftSteerKrakenId: number;
  frontLeftEncoderId: number;
  frontRightDriveKrakenId: number;
  frontRightSteerKrakenId: number;
  frontRightEncoderId: number;
  backLeftDriveKrakenId: number;
  backLeftSteerKrakenId: number;
  backLeftEncoderId: number;
  backRightDriveKrakenId: number;
  backRightSteerKrakenId: number;
  backRightEncoderId: number;
  kPigeonId: number;
  turretKrakenId: number;
  indexerKrakenId: number;
  hopperKrakenId: number;
  hoodKrakenId: number;
  homingSwitchCANdiID: number;
  intakePivotMotorId: number;
  intakeRollersLeadMotorId: number;
  intakeRollersFollowerMotorId: number;
  shooterLeaderId: number;
  shooterFollowerId: number;
}

export interface FeatureFlags {
  runDrive: boolean;
  useMAPoseEstimator: boolean;
  runVision: boolean;
  runHopper: boolean;
  runIndexer: boolean;
  runShooter: boolean;
  runTurret: boolean;
  runIntake: boolean;
  runHood: boolean;
  useHomingSwitch: boolean;
  useTuningServer: boolean;
  pretendCamerasAreMobile: boolean;
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

export interface DriveConstants {
  maxLinearSpeed: Units.LinearVelocity;
  maxAngularSpeed: Units.AngularVelocity;
  joystickDeadband: number;
  joystickMagnitudeExponent: number;
  linearDriveProfileMaxLinearVelocity: Units.LinearVelocity;
  linearDriveProfileMaxAngularVelocity: Units.AngularVelocity;
  linearDriveProfileMaxLinearAcceleration: Units.LinearAcceleration;
  linearDriveProfileMaxAngularAcceleration: Units.AngularAcceleration;
  linearDriveMaxAngularError: Units.Angle;
  linearDriveMaxPositionError: Units.Distance;
  linearDriveMaxLinearVelocityError: Units.LinearVelocity;
  linearDriveMaxAngularVelocityError: Units.AngularVelocity;
  steerGains: PIDGains;
  driveGains: PIDGains;
  defaultAutoPilotHeadingGains: PIDGains;
  defaultAutoPilotBeelineRadius: Units.Distance;
  defaultAutoPilotHeadingTolerance: Units.Angle;
  defaultAutoPilotXYTolerance: Units.Distance;
}

export interface VisionGainConstants {
  maxAcceptedDistanceMeters: number;
  linearStdDevFactor: number;
  angularStdDevFactor: number;
  maxZCutoff: number;
  maxSingleTagAmbiguity: number;
  maxAmbiguity: number;
}

export interface VisionConstants {
  gainConstants: VisionGainConstants;
  camera0Index: number;
  camera0Name: string;
  camera0Transform: Transform3d;
  camera1Index: number;
  camera1Name: string;
  camera1Transform: Transform3d;
  camera2Index: number;
  camera2Name: string;
  camera2Transform: Transform3d;
}

export type ClosedLoopOutputType = "Voltage" | "TorqueCurrentFOC";

export type StaticFeedforwardSignValue = "UseVelocitySign" | "UseClosedLoopSign";

export interface DrivetrainMotorConfig {
  closedLoopOutputType: ClosedLoopOutputType;
  gearRatio: number;
  momentOfInertia: Units.MomentOfInertia;
  frictionVoltage: Units.Voltage;
  staticFeedforwardSignValue: StaticFeedforwardSignValue;
}

export type SteerFeedbackType = "FusedCANcoder" | "SyncCANcoder" | "RemoteCANcoder" | "FusedCANdiPWM1" | "FusedCANdiPWM2" | "SyncCANdiPWM1" | "SyncCANdiPWM2" | "RemoteCANdiPWM1" | "RemoteCANdiPWM2" | "TalonFXS_PulseWidth";

export type DriveMotorArrangement = "TalonFX_Integrated" | "TalonFXS_NEO_JST" | "TalonFXS_VORTEX_JST";

export type SteerMotorArrangement = "TalonFX_Integrated" | "TalonFXS_Minion_JST" | "TalonFXS_NEO_JST" | "TalonFXS_VORTEX_JST" | "TalonFXS_NEO550_JST" | "TalonFXS_Brushed_AB" | "TalonFXS_Brushed_AC" | "TalonFXS_Brushed_BC";

export interface ModuleConfig {
  encoderOffset: Units.Angle;
  steerMotorInverted: boolean;
  encoderInverted: boolean;
  xPos: Units.Distance;
  yPos: Units.Distance;
}

export interface PhysicalDriveConstants {
  wheelRadius: Units.Distance;
  driveMotorConfig: DrivetrainMotorConfig;
  steerMotorConfig: DrivetrainMotorConfig;
  kSteerFeedbackType: SteerFeedbackType;
  kDriveMotorType: DriveMotorArrangement;
  kSteerMotorType: SteerMotorArrangement;
  frontLeftModule: ModuleConfig;
  frontRightModule: ModuleConfig;
  backLeftModule: ModuleConfig;
  backRightModule: ModuleConfig;
  kSlipCurrent: Units.Current;
  kSpeedAt12Volts: Units.LinearVelocity;
  kCoupleRatio: number;
  kInvertLeftSide: boolean;
  kInvertRightSide: boolean;
}

export interface OperatorConstants {
  controllerBindingsFile: string;
}

export type InvertedValue = "CounterClockwise_Positive" | "Clockwise_Positive";

export interface HopperConstants {
  dejamVoltage: Units.Voltage;
  hopperDemoMode: boolean;
  hopperReduction: number;
  spinningMovementThreshold: Units.AngularVelocity;
  hopperKP: number;
  hopperKI: number;
  hopperKD: number;
  hopperKS: number;
  hopperKV: number;
  hopperKG: number;
  hopperKA: number;
  hopperMaxAccelerationRotationsPerSecondSquared: number;
  hopperMaxJerkRotationsPerSecondCubed: number;
  hopperSupplyCurrentLimit: Units.Current;
  hopperStatorCurrentLimit: Units.Current;
  dejamCurrentThreshold: Units.Current;
  hopperMotorDirection: InvertedValue;
  simHopperMOI: Units.MomentOfInertia;
}

export interface IndexerConstants {
  indexerReduction: number;
  indexerMaximumRelativeVelocityError: number;
  indexerDemoMode: boolean;
  indexerKP: number;
  indexerKI: number;
  indexerKD: number;
  indexerKS: number;
  indexerKV: number;
  indexerKA: number;
  indexerMaxAccelerationRotationsPerSecondSquared: number;
  indexerMaxJerkRotationsPerSecondCubed: number;
  indexerSupplyCurrentLimit: Units.Current;
  indexerStatorCurrentLimit: Units.Current;
  simIndexerMOI: Units.MomentOfInertia;
  indexerMotorDirection: InvertedValue;
}

export interface TurretConstants {
  homingVoltage: Units.Voltage;
  turretReduction: number;
  homingMovementThreshold: Units.AngularVelocity;
  homingMaxUnmovingTime: Units.Time;
  homingAngle: Units.Angle;
  turretKP: number;
  turretKI: number;
  turretKD: number;
  turretKS: number;
  turretKV: number;
  turretKG: number;
  turretKA: number;
  turretExpoKV: number;
  turretExpoKA: number;
  turretSupplyCurrentLimit: Units.Current;
  turretStatorCurrentLimit: Units.Current;
  turretMotorDirection: InvertedValue;
  simTurretMOI: Units.MomentOfInertia;
  minTurretAngle: Units.Angle;
  maxTurretAngle: Units.Angle;
  headingToTurretAngle: Units.Angle;
}

export interface IntakeConstants {
  pivotReduction: number;
  rollersReduction: number;
  armLength: Units.Distance;
  minPivotAngle: Units.Angle;
  maxPivotAngle: Units.Angle;
  pivotStartingAngle: Units.Angle;
  rollersInertia: Units.MomentOfInertia;
  pivotInertia: Units.MomentOfInertia;
  intakePositionAngle: Units.Angle;
  stowPositionAngle: Units.Angle;
  intakeRollerSpeed: Units.AngularVelocity;
  homingMovementThreshold: Units.AngularVelocity;
  homingTimeoutSeconds: Units.Time;
  homingVoltage: Units.Voltage;
  pivotPIDGains: PIDGains;
  rollersPIDGains: PIDGains;
  pivotSupplyCurrentLimit: Units.Current;
  pivotStatorCurrentLimit: Units.Current;
  rollersStatorCurrentLimit: Units.Current;
  rollersSupplyCurrentLimit: Units.Current;
}

export type NeutralModeValue = "Coast" | "Brake";

export interface ShooterConstants {
  distanceToViDistancesMeters: number[];
  distanceToViVisMetersPerSecond: number[];
  defaultViMetersPerSecond: number;
  shooterKP: number;
  shooterKI: number;
  shooterKD: number;
  shooterKS: number;
  shooterKV: number;
  shooterKA: number;
  shooterReduction: number;
  shooterSupplyCurrentLimit: Units.Current;
  shooterStatorCurrentLimit: Units.Current;
  shooterMotorDirection: InvertedValue;
  invertFollower: boolean;
  defaultShooterNeutralMode: NeutralModeValue;
  shooterMaxVelocity: Units.AngularVelocity;
  shooterMaxAcceleration: Units.AngularAcceleration;
  shooterMOI: Units.MomentOfInertia;
}

export interface HoodConstants {
  disconnectedDebounceTimeSeconds: number;
  mechanismAngleToExitAngle: Units.Angle;
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
  hoodSupplyCurrentLimit: Units.Current;
  hoodStatorCurrentLimit: Units.Current;
  hoodMotorDirection: InvertedValue;
  minHoodAngle: Units.Angle;
  maxHoodAngle: Units.Angle;
  homingVoltage: Units.Voltage;
  homingMovementThreshold: Units.AngularVelocity;
  homingMaxUnmovingTime: Units.Time;
  simHoodMOI: Units.MomentOfInertia;
  simHoodArmLength: Units.Distance;
}

export interface ShotMapDataPoint {
  distance: Units.Distance;
  shooterRPM: number;
  hoodAngle: Units.Angle;
  flightTime: Units.Time;
}

export interface ShotMaps {
  hubDataPoints: ShotMapDataPoint[];
  passDataPoints: ShotMapDataPoint[];
  mechanismCompensationDelay: Units.Time;
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

export interface FieldLocationInstance {
  leftPassingTarget: Translation3d;
  rightPassingTarget: Translation3d;
  leftClimbLocation: Pose2d;
  rightClimbLocation: Pose2d;
}

export interface LowLevelButton {
  elementType: "button";
  id: string;
  inverted: boolean;
  minValue: number;
  maxValue: number;
  clampValue: boolean;
}

export interface LowLevelAxis {
  elementType: "axis";
  deadband: number;
  remapDeadband: boolean;
  id: string;
  inverted: boolean;
  minValue: number;
  maxValue: number;
  clampValue: boolean;
}

export interface LowLevelPOV {
  elementType: "pov";
  id: string;
  inverted: boolean;
  minValue: number;
  maxValue: number;
  clampValue: boolean;
}

export type LowLevelControlElement = LowLevelButton | LowLevelAxis | LowLevelPOV;

export interface Button {
  commandType: "button";
  threshold: number;
  thresholdRange: number;
  hysteresis: number;
  isToggle: boolean;
  hapticControlElement: LowLevelControlElement;
  command: string;
  inverted: boolean;
  minValue: number;
  maxValue: number;
  clampValue: boolean;
}

export interface Axis {
  commandType: "axis";
  hapticControlElement: LowLevelControlElement;
  command: string;
  inverted: boolean;
  minValue: number;
  maxValue: number;
  clampValue: boolean;
}

export interface POV {
  commandType: "pov";
  hapticControlElement: LowLevelControlElement;
  command: string;
  inverted: boolean;
  minValue: number;
  maxValue: number;
  clampValue: boolean;
}

export type ControlElement = Button | Axis | POV;

export interface Controller {
  port: number;
  type: string;
  controlElements: ControlElement[];
  buttonShorthands: { [key: string]: number };
  axisShorthands: { [key: string]: number };
  povShorthands: { [key: string]: number };
}

export interface Controllers {
  controllers: Controller[];
}

