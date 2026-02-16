package frc.robot.subsystems.drive.control_methods;

import coppercore.wpilib_interface.DriveWithJoysticks;
import frc.robot.subsystems.drive.Drive;

public class JoystickDrive extends DriveControlMethod {

  private DriveWithJoysticks command;

  public JoystickDrive(Drive drive, DriveWithJoysticks command) {
    super(drive, "JoystickDrive", true);
    this.command = command;
  }

  @Override
  public void activateControl() {
    command.initialize();
  }

  @Override
  protected void _periodic() {
    command.execute();
  }

  @Override
  public void deactivateControl() {
    command.end(false);
  }
}
