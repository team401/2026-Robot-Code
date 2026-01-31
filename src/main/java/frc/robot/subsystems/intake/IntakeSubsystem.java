package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TestModeManager;

public class IntakeSubsystem extends SubsystemBase {
  // TODO: add test mode support
  
  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("Intake", TestMode.class);

  private MotorIO pivotIO;
  private MotorIO rollersIO;

  private MotorInputsAutoLogged pivotInputs;
  private MotorInputsAutoLogged rollersInputs;

  public IntakeSubsystem(MotorIO pivotIO, MotorIO rollersIO) {
    this.pivotIO = pivotIO;
    this.rollersIO = rollersIO;

    this.pivotInputs = new MotorInputsAutoLogged();
    this.rollersInputs = new MotorInputsAutoLogged();
  }

  public void runRollers(double speedRPM) {
    rollersIO.controlToVelocityUnprofiled(Units.RPM.of(speedRPM));
  }

  public void stopRollers() {
    runRollers(0.0);
  }

  public void setTargetPivotAngle(Angle angle) {
    pivotIO.controlToPositionUnprofiled(angle);
  }

  public void setTargetPivotAngle(double angleRadians) {
    setTargetPivotAngle(Units.Radians.of(angleRadians));
  }

  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    rollersIO.updateInputs(rollersInputs);

    Logger.processInputs("intake/pivot/inputs", pivotInputs);
    Logger.processInputs("intake/rollers/inputs", rollersInputs);
  }

  public void testPeriodic() {
    return;
  }
}
