package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.drive.control_methods.LinearDrive;

public class DrivingLinearPathState extends State<DriveCoordinator> {

  protected LinearDrive.LinearDriveCommand command;

  public void setCommand(
      StateMachine<DriveCoordinator> stateMachine,
      DriveCoordinator world,
      LinearDrive.LinearDriveCommand command) {
    this.command = command;
    if (stateMachine.getCurrentState() == this) {
      // If we're currently in this state, update the command immediately
      world.LINEAR_DRIVE.setCommand(command);
    }
  }

  @Override
  protected void onEntry(StateMachine<DriveCoordinator> stateMachine, DriveCoordinator world) {
    world.LINEAR_DRIVE.setCommand(command);
  }

  @Override
  protected void periodic(StateMachine<DriveCoordinator> stateMachine, DriveCoordinator world) {
    world.setControlMethod(world.LINEAR_DRIVE);
    if (world.LINEAR_DRIVE.isFinished()) {
      finish();
      world.stateFinishAction(DriveCoordinator.DriveAction.DriveLinearPath);
    }
  }

  @Override
  protected void onExit(StateMachine<DriveCoordinator> stateMachine, DriveCoordinator world) {
    world.LINEAR_DRIVE.clearCommand();
    world.targetAction = null;
  }
}
