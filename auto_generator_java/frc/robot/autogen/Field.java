package frc.robot.autogen;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JsonConstants;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Wires the robot's {@link FieldConstants} to work headlessly inside the generator.
 *
 * <p>{@code FieldConstants.Tower.leftUpright()} reads {@code JsonConstants.aprilTagConstants}, whose
 * {@code getTagLayout()} normally calls {@code Filesystem.getDeployDirectory()} (which loads the HAL
 * native library). The generator runs under a bare JVM with no HAL, so instead we load the
 * AprilTag layout directly from a repo-relative path — honoring the {@code fieldType} deployment
 * constant — and inject it as the cached layout via reflection. This reuses all robot geometry with
 * no duplication and no HAL.
 */
public final class Field {
  private Field() {}

  private static final Path DEPLOY = Path.of("src", "main", "deploy");

  // climb_offset from the old constants.py: translate (0.41, 0.0225), rotate -90 degrees.
  private static final Transform2d CLIMB_OFFSET =
      new Transform2d(new Translation2d(0.41, 0.0225), Rotation2d.fromDegrees(-90.0));

  /** Loads the AprilTag layout for the given environment and installs it for FieldConstants. */
  public static void initialize(String environment) {
    String fileName = resolveLayoutFileName(environment);
    Path layoutPath = DEPLOY.resolve("apriltags").resolve(fileName);
    try {
      AprilTagFieldLayout layout = new AprilTagFieldLayout(layoutPath);
      AprilTagConstants constants = new AprilTagConstants();
      java.lang.reflect.Field cached = AprilTagConstants.class.getDeclaredField("cachedLayout");
      cached.setAccessible(true);
      cached.set(constants, layout);
      JsonConstants.aprilTagConstants = constants;
    } catch (ReflectiveOperationException | java.io.IOException e) {
      throw new RuntimeException("Failed to load AprilTag layout from " + layoutPath, e);
    }
  }

  private static String resolveLayoutFileName(String environment) {
    Path constantsFile = DEPLOY.resolve("constants").resolve(environment).resolve("AprilTagConstants.json");
    try {
      if (Files.exists(constantsFile)) {
        JsonObject json = JsonParser.parseString(Files.readString(constantsFile)).getAsJsonObject();
        if (json.has("fieldType")) {
          return AprilTagConstants.FieldType.valueOf(json.get("fieldType").getAsString())
              .getJsonFilename();
        }
      }
    } catch (Exception e) {
      System.err.println("[autogen] Could not read " + constantsFile + ": " + e);
    }
    // Fall back to the class default field type.
    return new AprilTagConstants().fieldType.getJsonFilename();
  }

  public static Pose2d leftClimbLocation() {
    return new Pose2d(FieldConstants.Tower.leftUpright(), new Rotation2d()).transformBy(CLIMB_OFFSET);
  }

  public static Pose2d rightClimbLocation() {
    return new Pose2d(FieldConstants.Tower.rightUpright(), new Rotation2d())
        .transformBy(CLIMB_OFFSET);
  }
}
