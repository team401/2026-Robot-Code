package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.drive.control_methods.LinearDrive;

public class DrivingLinearPathState extends State<DriveCoordinator> {

  protected LinearDrive.LinearDriveCommand command;

  public DrivingLinearPathState() {
    super("DrivingLinearPath");
  }

  public DrivingLinearPathState(String name) {
    super(name);
  }

  // This should only be called by the DriveCoordinator when it wants to set the command for this
  // state
  // before it enters this state, or by a subclass of this state that wants to update the command
  // based
  // before the onEntry method is called. If this is called while we're already in this state
  // then it will not update the command until we exit and re-enter this state, so it should not be
  // called
  // by any code that is trying to update the command while we're already in this state.
  public void setCommand(LinearDrive.LinearDriveCommand command) {
    this.command = command;
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
    }
  }

  @Override
  protected void onExit(StateMachine<DriveCoordinator> stateMachine, DriveCoordinator world) {
    world.LINEAR_DRIVE.clearCommand();
  }
}
