package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CoordinationLayer;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DriveCoordinator;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import org.json.simple.parser.ParseException;

public class Autos {

  public HashMap<String, Auto> autos = new HashMap<>();
  public HashMap<String, AutoAction> routines = new HashMap<>();

  @JSONExclude public HashMap<String, PathPlannerPath> paths = new HashMap<>();
  @JSONExclude public HashMap<String, Command> autoCommands = new HashMap<>();
  @JSONExclude public HashMap<String, Command> routineCommands = new HashMap<>();

  private static final String unFlippedSuffix = " Blue";
  private static final String flippedSuffix = " Red";
  private static final String unMirroredSuffix = " Left";
  private static final String mirroredSuffix = " Right";

  public void loadPathPlannerPath(String name) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      paths.put(name, path);
    } catch (FileVersionException | IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  public void loadAllPathPlannerPaths() {
    Path pathDirectory = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/paths");
    try {
      var pathFiles =
          java.nio.file.Files.list(pathDirectory).filter(p -> p.toString().endsWith(".path"));
      pathFiles.forEach(p -> loadPathPlannerPath(p.getFileName().toString().replace(".path", "")));
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void loadAutoCommands(
      DriveCoordinator driveCoordinator, CoordinationLayer coordinationLayer) {
    var context = new AutoAction.AutoActionContext(driveCoordinator, coordinationLayer, this);
    for (var entry : autos.entrySet()) {
      var baseName = entry.getKey();
      var auto = entry.getValue();
      var base_suffix =
          (auto.shouldFlip() ? unFlippedSuffix : "") + (auto.canMirror() ? unMirroredSuffix : "");
      autoCommands.put(baseName + base_suffix, entry.getValue().toCommand(context));
      if (auto.canMirror()) {
        autoCommands.put(
            entry.getKey() + (auto.shouldFlip() ? unFlippedSuffix : "") + mirroredSuffix,
            entry.getValue().toCommand(context.mirror()));
        if (auto.shouldFlip()) {
          autoCommands.put(
              entry.getKey() + flippedSuffix + mirroredSuffix,
              entry.getValue().toCommand(context.flip().mirror()));
        }
      }
      if (auto.shouldFlip()) {
        autoCommands.put(
            entry.getKey() + flippedSuffix + (auto.canMirror() ? unMirroredSuffix : ""),
            entry.getValue().toCommand(context.flip()));
      }
    }
    for (var entry : routines.entrySet()) {
      routineCommands.put(entry.getKey(), entry.getValue().toCommand(context));
    }
  }

  public Command getAutoCommand(String name) {
    return autoCommands.get(name);
  }

  public Command getRoutineCommand(String name) {
    return routineCommands.get(name);
  }

  public PathPlannerPath getPath(String name) {
    return paths.get(name);
  }

  public Command getRoutineCommandReference(String name) {
    return new Command() {
      public Command command = null;

      @Override
      public void initialize() {
        command = getRoutineCommand(name);
        if (command == null) {
          throw new RuntimeException("Routine with name '" + name + "' not found.");
        }
        command.initialize();
      }

      @Override
      public void execute() {
        if (command == null) {
          throw new RuntimeException("Command for routine '" + name + "' not initialized.");
        }
        command.execute();
      }

      @Override
      public boolean isFinished() {
        if (command == null) {
          throw new RuntimeException("Command for routine '" + name + "' not initialized.");
        }
        return command.isFinished();
      }

      @Override
      public void end(boolean interrupted) {
        if (command == null) {
          throw new RuntimeException("Command for routine '" + name + "' not initialized.");
        }
        command.end(interrupted);
      }
    };
  }

  public static Pose2d flipPose2d(Pose2d pose) {
    return FlippingUtil.flipFieldPose(pose);
  }

  public static Pose2d mirrorPose2d(Pose2d pose) {
    return new Pose2d(pose.getX(), FieldConstants.fieldWidth() - pose.getY(), mirrorRotation2d(pose.getRotation()));
  }

  public static Rotation2d flipRotation2d(Rotation2d rotation) {
    return FlippingUtil.flipFieldRotation(rotation);
  }

  public static Rotation2d mirrorRotation2d(Rotation2d rotation) {
    return Rotation2d.kPi.minus(rotation);
  }
}
