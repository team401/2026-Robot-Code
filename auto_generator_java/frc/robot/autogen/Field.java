package frc.robot.autogen;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JsonConstants;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Sets up the robot environment the generator runs in and exposes the field-derived climb poses.
 *
 * <p>The generator runs under a JVM that has the WPILib native libraries on its library path (see
 * generate.sh), so it can use robot code that touches HAL/Filesystem just like the real robot. We
 * initialize HAL and load {@link AprilTagConstants} for the active environment, then reuse {@link
 * FieldConstants} directly — no reflection, no geometry duplication.
 */
public final class Field {
  private Field() {}

  // climb_offset from the old constants.py: translate (0.41, 0.0225), rotate -90 degrees.
  private static final Transform2d CLIMB_OFFSET =
      new Transform2d(new Translation2d(0.41, 0.0225), Rotation2d.fromDegrees(-90.0));

  /** Initializes HAL and loads the AprilTag layout (honoring the fieldType deploy constant). */
  public static void initialize(String environment) {
    HAL.initialize(500, 0);

    Path constantsFile =
        Filesystem.getDeployDirectory()
            .toPath()
            .resolve("constants")
            .resolve(environment)
            .resolve("AprilTagConstants.json");

    AprilTagConstants constants;
    if (Files.exists(constantsFile)) {
      JSONSync<AprilTagConstants> sync =
          new JSONSync<>(
              new AprilTagConstants(),
              constantsFile.toString(),
              new JSONSyncConfigBuilder().build());
      sync.loadData();
      constants = sync.getObject();
    } else {
      // No environment-specific override; fall back to the class default field type.
      constants = new AprilTagConstants();
    }
    JsonConstants.aprilTagConstants = constants;
  }

  public static Pose2d leftClimbLocation() {
    return new Pose2d(FieldConstants.Tower.leftUpright(), new Rotation2d())
        .transformBy(CLIMB_OFFSET);
  }

  public static Pose2d rightClimbLocation() {
    return new Pose2d(FieldConstants.Tower.rightUpright(), new Rotation2d())
        .transformBy(CLIMB_OFFSET);
  }
}
