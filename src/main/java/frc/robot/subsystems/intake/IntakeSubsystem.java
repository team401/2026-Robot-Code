package frc.robot.subsystems.intake;

import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TestModeManager;

public class IntakeSubsystem extends SubsystemBase {
  // TODO: add test mode support
  // TODO: Move IntakeMechanism functionality into this class and remove IntakeMechanism class
  private final IntakeMechanism intakeMechanism;
  
  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("Intake", TestMode.class);

  public IntakeSubsystem(MotorIO pivotIO, MotorIO rollersIO) {
    intakeMechanism = new IntakeMechanism(pivotIO, rollersIO);
  }

  public void setTargetPivotAngle(double angleRadians) {
    intakeMechanism.setTargetPivotAngle(angleRadians);
  }

  public void runRollers(double speedRPM) {
    intakeMechanism.runRollers(speedRPM);
  }

  public void stopRollers() {
    intakeMechanism.runRollers(0.0);
  }

  @Override
  public void periodic() {
    intakeMechanism.periodic();
  }

  public void testPeriodic() {
    intakeMechanism.testPeriodic();
  }
}
