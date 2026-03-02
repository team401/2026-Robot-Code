package frc.robot.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;

public class StopDriveAction extends AutoAction {

  @Override
  public Command toCommand(AutoActionContext data) {
    return DriveCoordinatorCommands.stopDrive(data.driveCoordinator());
  }
}
