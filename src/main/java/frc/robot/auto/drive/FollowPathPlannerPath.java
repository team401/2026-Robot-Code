package frc.robot.auto.drive;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import org.json.simple.parser.ParseException;

public class FollowPathPlannerPath extends DriveAutoAction {

  public String pathName;
  public boolean mirrorPath;

  public static RobotConfig config;
  public static PathFollowingController controller;
  private static final BooleanSupplier FALSE = () -> false;

  static {
    try {
      config = RobotConfig.fromGUISettings();
      controller =
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  @Override
  public Command toCommand(AutoActionContext context) {
    var path = context.autos().getPathPlannerPath(pathName);
    if (path == null) {
      throw new RuntimeException("Path not found: " + pathName);
    }
    if (context.flipped()) {
      path = path.flipPath();
    }
    if (mirrorPath != context.mirrored()) {
      path = path.mirrorPath();
    }
    var drive = context.driveCoordinator().drive;
    return DriveCoordinatorCommands.wrapCommand(
        context.driveCoordinator(),
        new FollowPathCommand(
            path,
            drive::getPose,
            drive::getChassisSpeeds,
            (speeds, feedforwards) -> drive.setGoalSpeeds(speeds, false),
            controller,
            config,
            FALSE));
  }
}
