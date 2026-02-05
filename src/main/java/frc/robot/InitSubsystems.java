package frc.robot;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import coppercore.vision.VisionIO;
import coppercore.vision.VisionIOPhotonReal;
import coppercore.vision.VisionIOPhotonSim;
import coppercore.vision.VisionLocalizer;
import coppercore.vision.VisionLocalizer.CameraConfig;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.motors.MotorIOReplay;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFX;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFXSim;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.HomingSwitch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.io.dio_switch.DigitalInputIOCANdi;
import frc.robot.util.io.dio_switch.DigitalInputIOCANdiSimNT;
import frc.robot.util.io.dio_switch.DigitalInputIOReplay;

/**
 * The InitSubsystems class contains static methods to instantiate each subsystem. It is separated
 * from RobotContainer to make robot initialization easier to read and maintain.
 */
public class InitSubsystems {
  private InitSubsystems() {}

  public static Drive initDriveSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        return new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(JsonConstants.physicalDriveConstants.FrontLeft),
            new ModuleIOTalonFX(JsonConstants.physicalDriveConstants.FrontRight),
            new ModuleIOTalonFX(JsonConstants.physicalDriveConstants.BackLeft),
            new ModuleIOTalonFX(JsonConstants.physicalDriveConstants.BackRight));

      // The ModuleIOTalonFXS implementation provides an example implementation for
      // TalonFXS controller connected to a CANdi with a PWM encoder. The
      // implementations
      // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
      // swerve
      // template) can be freely intermixed to support alternative hardware
      // arrangements.
      // Please see the AdvantageKit template documentation for more information:
      // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
      //
      // drive =
      // new Drive(
      // new GyroIOPigeon2(),
      // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
      // new ModuleIOTalonFXS(TunerConstants.FrontRight),
      // new ModuleIOTalonFXS(TunerConstants.BackLeft),
      // new ModuleIOTalonFXS(TunerConstants.BackRight));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIOSim(JsonConstants.physicalDriveConstants.FrontLeft),
            new ModuleIOSim(JsonConstants.physicalDriveConstants.FrontRight),
            new ModuleIOSim(JsonConstants.physicalDriveConstants.BackLeft),
            new ModuleIOSim(JsonConstants.physicalDriveConstants.BackRight));

      default:
        // Replayed robot, disable IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }
  }

  public static VisionLocalizer initVisionSubsystem(Drive drive) {
    var gainConstants = JsonConstants.visionConstants.gainConstants;
    AprilTagFieldLayout tagLayout = JsonConstants.aprilTagConstants.getTagLayout();
    switch (Constants.currentMode) {
      case REAL:
        return new VisionLocalizer(
            drive::addVisionMeasurement,
            tagLayout,
            gainConstants,
            CameraConfig.fixed(
                new VisionIOPhotonReal("Camera1"), JsonConstants.visionConstants.camera1Transform),
            CameraConfig.fixed(
                new VisionIOPhotonReal("Camera2"), JsonConstants.visionConstants.camera2Transform),
            CameraConfig.fixed(
                new VisionIOPhotonReal("Camera3"), JsonConstants.visionConstants.camera3Transform));

      case SIM:
        return new VisionLocalizer(
            drive::addVisionMeasurement,
            tagLayout,
            gainConstants,
            CameraConfig.fixed(
                new VisionIOPhotonSim("Camera1", drive::getPose, VisionLocalizer.CameraType.FIXED),
                JsonConstants.visionConstants.camera1Transform),
            CameraConfig.fixed(
                new VisionIOPhotonSim("Camera2", drive::getPose, VisionLocalizer.CameraType.FIXED),
                JsonConstants.visionConstants.camera2Transform),
            CameraConfig.fixed(
                new VisionIOPhotonSim("Camera3", drive::getPose, VisionLocalizer.CameraType.FIXED),
                JsonConstants.visionConstants.camera3Transform));
      default:
        return new VisionLocalizer(
            drive::addVisionMeasurement,
            tagLayout,
            gainConstants,
            CameraConfig.fixed(new VisionIO() {}, new Transform3d()),
            CameraConfig.fixed(new VisionIO() {}, new Transform3d()),
            CameraConfig.fixed(new VisionIO() {}, new Transform3d()));
    }
  }

  public static HopperSubsystem initHopperSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new HopperSubsystem(
            MotorIOTalonFX.newLeader(
                JsonConstants.hopperConstants.buildMechanismConfig(),
                JsonConstants.hopperConstants.buildTalonFXConfigs()));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new HopperSubsystem(
            MotorIOTalonFXSim.newLeader(
                JsonConstants.hopperConstants.buildMechanismConfig(),
                JsonConstants.hopperConstants.buildTalonFXConfigs(),
                JsonConstants.hopperConstants.buildHopperSim()));

      default:
        // Replayed robot, disable IO implementations
        return new HopperSubsystem(new MotorIOReplay() {});
    }
  }

  public static IndexerSubsystem initIndexerSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new IndexerSubsystem(
            MotorIOTalonFX.newLeader(
                JsonConstants.indexerConstants.buildMechanismConfig(),
                JsonConstants.indexerConstants.buildTalonFXConfigs()));
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        MechanismConfig config = JsonConstants.indexerConstants.buildMechanismConfig();
        return new IndexerSubsystem(
            MotorIOTalonFXSim.newLeader(
                    config,
                    JsonConstants.indexerConstants.buildTalonFXConfigs(),
                    JsonConstants.indexerConstants.buildIndexerSim())
                .withMotorType(MotorType.KrakenX44));
      default:
        // Replayed robot, disable IO implementations
        return new IndexerSubsystem(new MotorIOReplay());
    }
  }

  public static TurretSubsystem initTurretSubsystem(
      DependencyOrderedExecutor dependencyOrderedExecutor) {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new TurretSubsystem(
            dependencyOrderedExecutor,
            MotorIOTalonFX.newLeader(
                JsonConstants.turretConstants.buildMechanismConfig(),
                JsonConstants.turretConstants.buildTalonFXConfigs()));
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        MechanismConfig config = JsonConstants.turretConstants.buildMechanismConfig();
        return new TurretSubsystem(
            dependencyOrderedExecutor,
            MotorIOTalonFXSim.newLeader(
                    config,
                    JsonConstants.turretConstants.buildTalonFXConfigs(),
                    JsonConstants.turretConstants.buildTurretSim())
                .withMotorType(MotorType.KrakenX44));
      default:
        // Replayed robot, disable IO implementations
        return new TurretSubsystem(dependencyOrderedExecutor, new MotorIOReplay());
    }
  }

  public static HoodSubsystem initHoodSubsystem(
      DependencyOrderedExecutor dependencyOrderedExecutor) {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new HoodSubsystem(
            dependencyOrderedExecutor,
            MotorIOTalonFX.newLeader(
                JsonConstants.hoodConstants.buildMechanismConfig(),
                JsonConstants.hoodConstants.buildTalonFXConfigs()));
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        MechanismConfig config = JsonConstants.hoodConstants.buildMechanismConfig();
        return new HoodSubsystem(
            dependencyOrderedExecutor,
            MotorIOTalonFXSim.newLeader(
                    config,
                    JsonConstants.hoodConstants.buildTalonFXConfigs(),
                    JsonConstants.hoodConstants.buildHoodSim())
                .withMotorType(MotorType.KrakenX44));
      default:
        // Replayed robot, disable IO implementations
        return new HoodSubsystem(dependencyOrderedExecutor, new MotorIOReplay());
    }
  }

  public static HomingSwitch initHomingSwitch(DependencyOrderedExecutor dependencyOrderedExecutor) {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new HomingSwitch(
            dependencyOrderedExecutor,
            new DigitalInputIOCANdi(
                new CANDeviceID(
                    JsonConstants.robotInfo.CANBus,
                    JsonConstants.canBusAssignment.homingSwitchCANdiID),
                JsonConstants.robotInfo.buildHomingSwitchConfig(),
                JsonConstants.robotInfo.homingSwitchSignal));
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new HomingSwitch(
            dependencyOrderedExecutor,
            new DigitalInputIOCANdiSimNT(
                new CANDeviceID(
                    JsonConstants.robotInfo.CANBus,
                    JsonConstants.canBusAssignment.homingSwitchCANdiID),
                JsonConstants.robotInfo.buildHomingSwitchConfig(),
                JsonConstants.robotInfo.homingSwitchSignal,
                "HomingSwitchSim/isOpen",
                false));
      default:
        // Replayed robot, disable IO implementations
        return new HomingSwitch(dependencyOrderedExecutor, new DigitalInputIOReplay());
    }
  }

  public static IntakeSubsystem initIntakeSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new IntakeSubsystem(
            MotorIOTalonFX.newLeader(
                JsonConstants.intakeConstants.buildPivotMechanismConfig(),
                JsonConstants.intakeConstants.buildPivotTalonFXMotorConfig()),
            MotorIOTalonFX.newLeader(
                JsonConstants.intakeConstants.buildRollersMechanismConfig(),
                JsonConstants.intakeConstants.buildRollersTalonFXMotorConfig()),
            MotorIOTalonFX.newFollower(JsonConstants.intakeConstants.buildRollersMechanismConfig(),
            0,
                JsonConstants.intakeConstants.buildRollersTalonFXMotorConfig()));
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        MechanismConfig pivotConfig = JsonConstants.intakeConstants.buildPivotMechanismConfig();
        MechanismConfig rollersConfig = JsonConstants.intakeConstants.buildRollersMechanismConfig();
        return new IntakeSubsystem(
            MotorIOTalonFXSim.newLeader(
                    pivotConfig,
                    JsonConstants.intakeConstants.buildPivotTalonFXMotorConfig(),
                    JsonConstants.intakeConstants.buildPivotSim()),
            MotorIOTalonFXSim.newLeader(
                    rollersConfig,
                    JsonConstants.intakeConstants.buildRollersTalonFXMotorConfig(),
                    JsonConstants.intakeConstants.buildRollersSim()),
            MotorIOTalonFXSim.newFollower(
                    rollersConfig,
                    0,
                    JsonConstants.intakeConstants.buildRollersTalonFXMotorConfig()));
              
      default:
        // Replayed robot, disable IO implementations
        return new IntakeSubsystem(new MotorIOReplay(), new MotorIOReplay(), new MotorIOReplay());
    }
  }
}
