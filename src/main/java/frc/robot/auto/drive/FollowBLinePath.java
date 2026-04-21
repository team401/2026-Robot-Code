package frc.robot.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;

public class FollowBLinePath extends AutoAction {
  public String pathName;
  public boolean mirrorPath;

  @Override
  public Command toCommand(AutoActionContext context) {
    Path path;
    try {
      path = new Path(pathName);
    } catch (RuntimeException e) {
      throw new RuntimeException("B-Line Path not found: " + pathName + "\nError:\n" + e);
    }

    if (context.flipped()) {
      path.flip();
    }
    if (mirrorPath != context.mirrored()) {
      path.mirror();
    }

    return DriveCoordinatorCommands.followBLinePath(context.driveCoordinator(), path);
  }
}
