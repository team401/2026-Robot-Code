package frc.robot.auto.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class FollowPathPlannerPath extends DriveAutoAction {

  public String pathName;
  public boolean flipPath;
  public boolean mirrorPath;

  public static RobotConfig config;

  static {
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  @Override
  public Command toCommand(AutoActionContext context) {
    var path = context.autos().getPath(pathName);
    if (flipPath) {
      path = path.flipPath();
    }
    if (mirrorPath) {
      path = path.mirrorPath();
    }
    // To make it so that this is effectively final
    var _path = path;
    return wrapCommand(
        context,
        new Command() {

          PathPlannerTrajectory trajectory;
          double timeSeconds;

          @Override
          public void initialize() {
            trajectory =
                _path.generateTrajectory(
                    context.driveCoordinator().drive.getChassisSpeeds(),
                    context.driveCoordinator().drive.getRotation(),
                    config);
            timeSeconds = 0;
            AutoBuilder
          }

          @Override
          public void execute() {
            var state = trajectory.sample(timeSeconds);
            timeSeconds += 0.02; // Cycle time
            context.driveCoordinator().drive.setGoalSpeedsBlueOrigins(state.fieldSpeeds);
          }

          @Override
          public boolean isFinished() {
            return timeSeconds > trajectory.getTotalTimeSeconds();
          }
        });
  }
}
