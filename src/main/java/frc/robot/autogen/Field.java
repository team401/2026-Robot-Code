package frc.robot.autogen;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JsonConstants;

/**
 * Sets up the robot environment the generator runs in and exposes the field-derived climb poses.
 *
 * <p>The generator runs under a JVM that has the WPILib native libraries on its library path (see
 * generate.py), so it can use robot code that touches HAL/Filesystem just like the real robot. We
 * initialize HAL and then run the robot's own constant-loading sequence ({@link
 * JsonConstants#loadConstants()}), which makes every constant (FieldConstants, AprilTagConstants,
 * drive constants, ...) available exactly as on the robot — no duplicated loading logic.
 */
public final class Field {
  private Field() {}

  // climb_offset from the old constants.py: translate (0.41, 0.0225), rotate -90 degrees.
  private static final Transform2d CLIMB_OFFSET =
      new Transform2d(new Translation2d(0.41, 0.0225), Rotation2d.fromDegrees(-90.0));

  /** Initializes HAL and loads all robot constants for the active (config.json) environment. */
  public static void initialize() {
    HAL.initialize(500, 0);
    JsonConstants.loadConstants();
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
