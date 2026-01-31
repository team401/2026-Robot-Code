package frc.robot.subsystems.drive.states;

import coppercore.wpilib_interface.DriveWithJoysticks;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CommandState;

public class DriveWithJoysticksState extends CommandState<Drive> {

  // This is to allow subclasses (DriveTestModeState) to set the name

  protected DriveWithJoysticksState(String name, DriveWithJoysticks driveCommand) {
    super(name, driveCommand);
  }

  public DriveWithJoysticksState(DriveWithJoysticks driveCommand) {
    this("DriveWithJoysticks", driveCommand);
  }
  
}
