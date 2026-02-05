package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.DriveWithJoysticks;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.util.LoggedTunablePIDGains;

// Extends DriveWithJoysticksState to allow you to drive the robot while tuning
// the drive and steer gains
public class DriveTestModeState extends DriveWithJoysticksState {

  LoggedTunablePIDGains steerGains;
  LoggedTunablePIDGains driveGains;

  public DriveTestModeState(Drive drive, DriveWithJoysticks driveCommand) {
    super("Drive Test Mode", driveCommand);

    steerGains =
        new LoggedTunablePIDGains(
            "Drive Test Mode/Steer Gains", JsonConstants.driveConstants.steerGains.asArray());
    driveGains =
        new LoggedTunablePIDGains(
            "Drive Test Mode/Drive Gains", JsonConstants.driveConstants.driveGains.asArray());
  }

  @Override
  public void periodic(StateMachine<Drive> stateMachine, Drive drive) {
    switch (DriveCoordinator.getTestModeManager().getTestMode()) {
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
