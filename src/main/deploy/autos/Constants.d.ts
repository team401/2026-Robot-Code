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
  maxLinearSpeed: JSONMeasure;
  maxAngularSpeed: JSONMeasure;
  joystickDeadband: number;
  joystickMagnitudeExponent: number;
  linearDriveProfileMaxLinearVelocity: JSONMeasure;
  linearDriveProfileMaxAngularVelocity: JSONMeasure;
  linearDriveProfileMaxLinearAcceleration: JSONMeasure;
  linearDriveProfileMaxAngularAcceleration: JSONMeasure;
  linearDriveMaxAngularError: JSONMeasure;
  linearDriveMaxPositionError: JSONMeasure;
  linearDriveMaxLinearVelocityError: JSONMeasure;
  linearDriveMaxAngularVelocityError: JSONMeasure;
  steerGains: PIDGains;
  driveGains: PIDGains;
  defaultAutoPilotHeadingGains: PIDGains;
  defaultAutoPilotBeelineRadius: JSONMeasure;
  defaultAutoPilotHeadingTolerance: JSONMeasure;
  defaultAutoPilotXYTolerance: JSONMeasure;
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
  camera0Transform: JSONTransform3d;
  camera1Index: number;
  camera1Name: string;
  camera1Transform: JSONTransform3d;
  camera2Index: number;
  camera2Name: string;
  camera2Transform: JSONTransform3d;
}

export type ClosedLoopOutputType = "Voltage" | "TorqueCurrentFOC";

export type StaticFeedforwardSignValue = "UseVelocitySign" | "UseClosedLoopSign";

export interface DrivetrainMotorConfig {
  closedLoopOutputType: ClosedLoopOutputType;
  gearRatio: number;
  momentOfInertia: JSONMeasure;
  frictionVoltage: JSONMeasure;
  staticFeedforwardSignValue: StaticFeedforwardSignValue;
}

export type SteerFeedbackType = "FusedCANcoder" | "SyncCANcoder" | "RemoteCANcoder" | "FusedCANdiPWM1" | "FusedCANdiPWM2" | "SyncCANdiPWM1" | "SyncCANdiPWM2" | "RemoteCANdiPWM1" | "RemoteCANdiPWM2" | "TalonFXS_PulseWidth";

export type DriveMotorArrangement = "TalonFX_Integrated" | "TalonFXS_NEO_JST" | "TalonFXS_VORTEX_JST";

export type SteerMotorArrangement = "TalonFX_Integrated" | "TalonFXS_Minion_JST" | "TalonFXS_NEO_JST" | "TalonFXS_VORTEX_JST" | "TalonFXS_NEO550_JST" | "TalonFXS_Brushed_AB" | "TalonFXS_Brushed_AC" | "TalonFXS_Brushed_BC";

export interface ModuleConfig {
  encoderOffset: JSONMeasure;
  steerMotorInverted: boolean;
  encoderInverted: boolean;
  xPos: JSONMeasure;
  yPos: JSONMeasure;
}

export interface PhysicalDriveConstants {
  wheelRadius: JSONMeasure;
  driveMotorConfig: DrivetrainMotorConfig;
  steerMotorConfig: DrivetrainMotorConfig;
  kSteerFeedbackType: SteerFeedbackType;
  kDriveMotorType: DriveMotorArrangement;
  kSteerMotorType: SteerMotorArrangement;
  frontLeftModule: ModuleConfig;
  frontRightModule: ModuleConfig;
  backLeftModule: ModuleConfig;
  backRightModule: ModuleConfig;
  kSlipCurrent: JSONMeasure;
  kSpeedAt12Volts: JSONMeasure;
  kCoupleRatio: number;
  kInvertLeftSide: boolean;
  kInvertRightSide: boolean;
}

export interface OperatorConstants {
  controllerBindingsFile: string;
}

export type InvertedValue = "CounterClockwise_Positive" | "Clockwise_Positive";

export interface HopperConstants {
  dejamVoltage: JSONMeasure;
  hopperDemoMode: boolean;
  hopperReduction: number;
  spinningMovementThreshold: JSONMeasure;
  hopperKP: number;
  hopperKI: number;
  hopperKD: number;
  hopperKS: number;
  hopperKV: number;
  hopperKG: number;
  hopperKA: number;
  hopperMaxAccelerationRotationsPerSecondSquared: number;
  hopperMaxJerkRotationsPerSecondCubed: number;
  hopperSupplyCurrentLimit: JSONMeasure;
  hopperStatorCurrentLimit: JSONMeasure;
  dejamCurrentThreshold: JSONMeasure;
  hopperMotorDirection: InvertedValue;
  simHopperMOI: JSONMeasure;
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
  indexerSupplyCurrentLimit: JSONMeasure;
  indexerStatorCurrentLimit: JSONMeasure;
  simIndexerMOI: JSONMeasure;
  indexerMotorDirection: InvertedValue;
}

export interface TurretConstants {
  homingVoltage: JSONMeasure;
  turretReduction: number;
  homingMovementThreshold: JSONMeasure;
  homingMaxUnmovingTime: JSONMeasure;
  homingAngle: JSONMeasure;
  turretKP: number;
  turretKI: number;
  turretKD: number;
  turretKS: number;
  turretKV: number;
  turretKG: number;
  turretKA: number;
  turretExpoKV: number;
  turretExpoKA: number;
  turretSupplyCurrentLimit: JSONMeasure;
  turretStatorCurrentLimit: JSONMeasure;
  turretMotorDirection: InvertedValue;
  simTurretMOI: JSONMeasure;
  minTurretAngle: JSONMeasure;
  maxTurretAngle: JSONMeasure;
  headingToTurretAngle: JSONMeasure;
}

export interface IntakeConstants {
  pivotReduction: number;
  rollersReduction: number;
  armLength: JSONMeasure;
  minPivotAngle: JSONMeasure;
  maxPivotAngle: JSONMeasure;
  pivotStartingAngle: JSONMeasure;
  rollersInertia: JSONMeasure;
  pivotInertia: JSONMeasure;
  intakePositionAngle: JSONMeasure;
  stowPositionAngle: JSONMeasure;
  intakeRollerSpeed: JSONMeasure;
  homingMovementThreshold: JSONMeasure;
  homingTimeoutSeconds: JSONMeasure;
  homingVoltage: JSONMeasure;
  pivotPIDGains: PIDGains;
  rollersPIDGains: PIDGains;
  pivotSupplyCurrentLimit: JSONMeasure;
  pivotStatorCurrentLimit: JSONMeasure;
  rollersStatorCurrentLimit: JSONMeasure;
  rollersSupplyCurrentLimit: JSONMeasure;
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
  shooterSupplyCurrentLimit: JSONMeasure;
  shooterStatorCurrentLimit: JSONMeasure;
  shooterMotorDirection: InvertedValue;
  invertFollower: boolean;
  defaultShooterNeutralMode: NeutralModeValue;
  shooterMaxVelocity: JSONMeasure;
  shooterMaxAcceleration: JSONMeasure;
  shooterMOI: JSONMeasure;
}

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

export interface ShotMapDataPoint {
  distance: JSONMeasure;
  shooterRPM: number;
  hoodAngle: JSONMeasure;
  flightTime: JSONMeasure;
}

export interface ShotMaps {
  hubDataPoints: ShotMapDataPoint[];
  passDataPoints: ShotMapDataPoint[];
  mechanismCompensationDelay: JSONMeasure;
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

export interface FieldLocationInstance {
  leftPassingTarget: JSONTranslation3d;
  rightPassingTarget: JSONTranslation3d;
  leftClimbLocation: JSONPose2d;
  rightClimbLocation: JSONPose2d;
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

export interface ControllerJsonRepresentation {
  port: number;
  type: string;
  controlElements: ControlElement[];
  buttonShorthands: { [key: string]: number };
  axisShorthands: { [key: string]: number };
  povShorthands: { [key: string]: number };
}

export interface Controllers {
  controllers: ControllerJsonRepresentation[];
}

