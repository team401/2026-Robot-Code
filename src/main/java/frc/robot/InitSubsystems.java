package frc.robot;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.motors.MotorIOReplay;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFX;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFXSim;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

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
            new ModuleIOTalonFX(JsonConstants.drivetrainConstants.FrontLeft),
            new ModuleIOTalonFX(JsonConstants.drivetrainConstants.FrontRight),
            new ModuleIOTalonFX(JsonConstants.drivetrainConstants.BackLeft),
            new ModuleIOTalonFX(JsonConstants.drivetrainConstants.BackRight));

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
            new ModuleIOSim(JsonConstants.drivetrainConstants.FrontLeft),
            new ModuleIOSim(JsonConstants.drivetrainConstants.FrontRight),
            new ModuleIOSim(JsonConstants.drivetrainConstants.BackLeft),
            new ModuleIOSim(JsonConstants.drivetrainConstants.BackRight));

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

  public static TurretSubsystem initTurretSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new TurretSubsystem(
            MotorIOTalonFX.newLeader(
                JsonConstants.turretConstants.buildMechanismConfig(),
                JsonConstants.turretConstants.buildTalonFXConfigs()));
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        MechanismConfig config = JsonConstants.turretConstants.buildMechanismConfig();
        return new TurretSubsystem(
            MotorIOTalonFXSim.newLeader(
                    config,
                    JsonConstants.turretConstants.buildTalonFXConfigs(),
                    JsonConstants.turretConstants.buildTurretSim())
                .withMotorType(MotorType.KrakenX44));
      default:
        // Replayed robot, disable IO implementations
        return new TurretSubsystem(new MotorIOReplay());
    }
  }
}
