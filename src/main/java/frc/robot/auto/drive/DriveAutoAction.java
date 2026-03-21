package frc.robot.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;
import frc.robot.util.ts.GeneratedOptional;

public abstract class DriveAutoAction extends AutoAction {

  // Just tells us if we can mirror for things
  // Because for things like climb we can not mirror the climb poses
  @GeneratedOptional public boolean canMirror = true;

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
