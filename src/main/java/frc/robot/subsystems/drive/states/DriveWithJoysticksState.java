package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import frc.robot.subsystems.drive.DriveCoordinator;

public class DriveWithJoysticksState extends State<DriveCoordinator> {

  // This is to allow subclasses (DriveTestModeState) to set the name
  public DriveWithJoysticksState() {
    super("DriveWithJoysticks");
  }

  @Override
  protected void periodic(StateMachine<DriveCoordinator> stateMachine, DriveCoordinator world) {
    world.setControlMethod(world.JOYSTICK_DRIVE);
  }
}
