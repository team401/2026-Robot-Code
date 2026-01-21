package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.StateMachine;
import frc.robot.TestMode;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunablePIDGains;
import java.util.Set;

// Extends DriveWithJoysticksState to allow you to drive the robot while tuning
// the drive and steer gains
public class DriveTestModeState extends DriveWithJoysticksState {

  private static final Set<TestMode> driveTestModes = Set.of(TestMode.DriveGainsTuning);

  // I put this method here so that all drive test code is in one place
  public static boolean isDriveTestMode() {
    return driveTestModes.contains(TestModeManager.getTestMode());
  }

  LoggedTunablePIDGains steerGains;
  LoggedTunablePIDGains driveGains;

  public DriveTestModeState(Drive drive) {
    super("Drive Test Mode");

    steerGains =
        new LoggedTunablePIDGains(
            "Drive Test Mode Steer Gains", JsonConstants.drivetrainConstants.steerGains.asArray());
    driveGains =
        new LoggedTunablePIDGains(
            "Drive Test Mode Drive Gains", JsonConstants.drivetrainConstants.driveGains.asArray());
  }

  @Override
  public void periodic(StateMachine<Drive> stateMachine, Drive drive) {
    switch (TestModeManager.getTestMode()) {
      case DriveGainsTuning:
        steerGains.ifChanged(hashCode(), drive::setSteerGains);
        driveGains.ifChanged(hashCode(), drive::setDriveGains);
        break;
      default:
        finish();
        break;
    }
    super.periodic(stateMachine, drive);
  }
}
