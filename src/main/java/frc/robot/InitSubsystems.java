package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.motors.MotorIOReplay;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFX;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFXSim;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
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
import frc.robot.subsystems.shooter.ShooterSubsystem;
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

      case REPLAY:
        // Replayed robot, disable IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
      default:
        throw new UnsupportedOperationException("Unsupported mode " + Constants.currentMode);
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

      case REPLAY:
        // Replayed robot, disable IO implementations
        return new HopperSubsystem(new MotorIOReplay() {});
      default:
        throw new UnsupportedOperationException("Unsupported mode " + Constants.currentMode);
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
      case REPLAY:
        // Replayed robot, disable IO implementations
        return new IndexerSubsystem(new MotorIOReplay());
      default:
        throw new UnsupportedOperationException("Unsupported mode " + Constants.currentMode);
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
      case REPLAY:
        // Replayed robot, disable IO implementations
        return new TurretSubsystem(dependencyOrderedExecutor, new MotorIOReplay());
      default:
        throw new UnsupportedOperationException("Unsupported mode " + Constants.currentMode);
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
      case REPLAY:
        // Replayed robot, disable IO implementations
        return new HoodSubsystem(dependencyOrderedExecutor, new MotorIOReplay());
      default:
        throw new UnsupportedOperationException("Unsupported mode " + Constants.currentMode);
    }
  }

  public static ShooterSubsystem initShooterSubsystem(
      DependencyOrderedExecutor dependencyOrderedExecutor) {
    MechanismConfig mechanismConfig = JsonConstants.shooterConstants.buildMechanismConfig();
    TalonFXConfiguration talonFXConfigs = JsonConstants.shooterConstants.buildTalonFXConfigs();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new ShooterSubsystem(
            dependencyOrderedExecutor,
            MotorIOTalonFX.newLeader(mechanismConfig, talonFXConfigs),
            MotorIOTalonFX.newFollower(mechanismConfig, 0, talonFXConfigs),
            MotorIOTalonFX.newFollower(mechanismConfig, 1, talonFXConfigs));
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        CoppercoreSimAdapter shooterSim = JsonConstants.shooterConstants.buildShooterSim();

        return new ShooterSubsystem(
            dependencyOrderedExecutor,
            MotorIOTalonFXSim.newLeader(mechanismConfig, talonFXConfigs, shooterSim),
            MotorIOTalonFXSim.newFollower(mechanismConfig, 0, talonFXConfigs, shooterSim),
            MotorIOTalonFXSim.newFollower(mechanismConfig, 1, talonFXConfigs, shooterSim));
      case REPLAY:
        // Replayed robot, disable IO implementations
        return new ShooterSubsystem(
            dependencyOrderedExecutor,
            new MotorIOReplay(),
            new MotorIOReplay(),
            new MotorIOReplay());
      default:
        throw new UnsupportedOperationException("Unsupported mode " + Constants.currentMode);
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
      case REPLAY:
        // Replayed robot, disable IO implementations
        return new HomingSwitch(dependencyOrderedExecutor, new DigitalInputIOReplay());
      default:
        throw new UnsupportedOperationException("Unsupported mode " + Constants.currentMode);
    }
  }
}
