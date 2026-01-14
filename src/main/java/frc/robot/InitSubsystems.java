package frc.robot;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

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
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

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
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

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
}
