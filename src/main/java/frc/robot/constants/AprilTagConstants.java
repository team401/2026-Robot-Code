package frc.robot.constants;

import coppercore.parameter_tools.json.annotations.JSONExclude;
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

  public final FieldType fieldType = FieldType.ANDYMARK;

  @JSONExclude private AprilTagFieldLayout cachedLayout;

  public AprilTagFieldLayout getTagLayout() {
    if (cachedLayout == null) {
      try {
        Path p =
            Path.of(
                Filesystem.getDeployDirectory().getPath(),
                "apriltags",
                fieldType.getJsonFilename());
        cachedLayout = new AprilTagFieldLayout(p);
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }

    return cachedLayout;
  }
}
