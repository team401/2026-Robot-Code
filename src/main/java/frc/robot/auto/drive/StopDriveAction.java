package frc.robot.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;

public class StopDriveAction extends AutoAction {

  @Override
  public Command toCommand(AutoActionContext data) {
    return wrapCommand(data, DriveCoordinatorCommands.stopDrive(data.driveCoordinator()));
  }

  public Command wrapCommand(AutoActionContext data, Command command) {
    return new Command() {
      @Override
      public void initialize() {
        data.driveCoordinator().setCurrentDriveCommand(command);
      }

      @Override
      public void end(boolean interrupted) {
        data.driveCoordinator().cancelCurrentDriveCommand();
      }

      @Override
      public boolean isFinished() {
        return command.isFinished();
      }
    };
  }
}
