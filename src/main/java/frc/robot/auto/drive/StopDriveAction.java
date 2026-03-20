package frc.robot.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;

public class StopDriveAction extends DriveAutoAction {

  @Override
  public Command toCommand(AutoActionContext data) {
    return wrapCommand(data, DriveCoordinatorCommands.stopDrive(data.driveCoordinator()));
  }
}
