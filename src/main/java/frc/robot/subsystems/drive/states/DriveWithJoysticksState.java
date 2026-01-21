package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.DriveWithJoysticks;
import frc.robot.subsystems.drive.Drive;

public class DriveWithJoysticksState extends State<Drive> {

  private DriveWithJoysticks driveCommand;

  // This is to allow subclasses (DriveTestModeState) to set the name
  protected DriveWithJoysticksState(String name) {
    super(name);
  }

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
  protected void onEntry(StateMachine<Drive> stateMachine, Drive world) {
    if (driveCommand != null) {
      driveCommand.initialize();
    }
  }

  @Override
  protected void periodic(StateMachine<Drive> stateMachine, Drive world) {
    driveCommand.execute();
  }

  @Override
  protected void onExit(StateMachine<Drive> stateMachine, Drive world) {
    if (driveCommand != null) {
      driveCommand.end(false);
    }
  }
}
