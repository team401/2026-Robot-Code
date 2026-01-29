package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

/**
 * AprilTagConstants provides a constant for deciding between AndyMark and Welded fields, and to
 * easily get the AprilTagFieldLayout from that choice.
 */
public class AprilTagConstants {
  public enum FieldType {
    ANDYMARK("2026-rebuilt-andymark.json"),
    WELDED("2026-rebuilt-welded.json");

    private final String jsonFilename;

    FieldType(String jsonFilename) {
      this.jsonFilename = jsonFilename;
    }

    public String getJsonFilename() {
      return jsonFilename;
    }
  }

  public final FieldType fieldType = FieldType.WELDED;

  public AprilTagFieldLayout getTagLayout() {
    try {
      Path p =
          Path.of(
              Filesystem.getDeployDirectory().getPath(), "apriltags", fieldType.getJsonFilename());
      AprilTagFieldLayout layout = new AprilTagFieldLayout(p);

      return layout;
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
