package frc.robot.subsystems.drive.control_methods;

import frc.robot.subsystems.drive.Drive;

// This control method is used to disable the drive
public class DisabledDrive extends DriveControlMethod {

  public DisabledDrive(Drive drive, String logPrefix) {
    super(drive, logPrefix, "DisabledDrive");
  }

  @Override
  protected void _periodic() {
    drive.stop();
  }
}
