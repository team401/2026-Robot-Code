package frc.robot.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;

public abstract class DriveAutoAction extends AutoAction {
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
