package frc.robot.auto.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;

public class FollowBLinePath extends DriveAutoAction {

  public String pathName;
  public boolean mirrorPath;

  @Override
  public Command toCommand(AutoActionContext context) {
    var path = context.autos().getBlinePath(pathName);
    if (path == null) {
      throw new RuntimeException("BLine Path not found: " + pathName);
    }
    if (context.flipped()) {
      path.flip();
    }
    if (mirrorPath != context.mirrored()) {
      path.mirror();
    }

    return DriveCoordinatorCommands.wrapCommand(
        context.driveCoordinator(), RobotContainer.getPathBuilder().build(path));
  }
}
