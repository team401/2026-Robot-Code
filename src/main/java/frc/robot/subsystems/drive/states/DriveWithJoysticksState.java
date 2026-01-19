package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.DriveWithJoysticks;
import frc.robot.subsystems.drive.Drive;

// Copilot used to help write this class

public class DriveWithJoysticksState extends State<Drive> {

  private DriveWithJoysticks driveCommand;

  public DriveWithJoysticksState() {
    super("DriveWithJoysticks");
  }

  public DriveWithJoysticks getDriveCommand() {
    return driveCommand;
  }

  public void setDriveCommand(DriveWithJoysticks driveCommand) {
    this.driveCommand = driveCommand;
  }

  @Override
  protected void periodic(StateMachine<Drive> stateMachine, Drive world) {
    driveCommand.execute();
  }
}
