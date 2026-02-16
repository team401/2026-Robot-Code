package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.util.LoggedTunablePIDGains;

// Extends DriveWithJoysticksState to allow you to drive the robot while tuning
// the drive and steer gains
public class DriveTestModeState extends State<DriveCoordinator> {

  LoggedTunablePIDGains steerGains;
  LoggedTunablePIDGains driveGains;

  public DriveTestModeState() {
    super("Drive Test Mode");

    steerGains =
        new LoggedTunablePIDGains(
            "Drive Test Mode/Steer Gains", JsonConstants.driveConstants.steerGains.asArray());
    driveGains =
        new LoggedTunablePIDGains(
            "Drive Test Mode/Drive Gains", JsonConstants.driveConstants.driveGains.asArray());
  }

  @Override
  public void periodic(StateMachine<DriveCoordinator> stateMachine, DriveCoordinator world) {
    switch (DriveCoordinator.getTestModeManager().getTestMode()) {
      case DriveGainsTuning:
        steerGains.ifChanged(hashCode(), world.drive::setSteerGains);
        driveGains.ifChanged(hashCode(), world.drive::setDriveGains);
        break;
      default:
        finish();
        break;
    }
    world.setControlMethod(world.JOYSTICK_DRIVE);
  }
}
