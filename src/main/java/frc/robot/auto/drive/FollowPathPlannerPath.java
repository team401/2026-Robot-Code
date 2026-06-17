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
  private static boolean configInitialized = false;
  private static final BooleanSupplier FALSE = () -> false;

  /**
   * Lazily loads the PathPlanner {@code RobotConfig} and controller on first use. This used to run
   * in a static initializer, but that pulled in NetworkTables/SmartDashboard at class-load time,
   * which prevents the class from loading in the headless Java auto generator. It is only needed
   * when actually building a command, so it is deferred to {@link #toCommand}.
   */
  private static void ensureConfigInitialized() {
    if (configInitialized) {
      return;
    }
    configInitialized = true;
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
    ensureConfigInitialized();
    var path = context.autos().getPath(pathName);
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
