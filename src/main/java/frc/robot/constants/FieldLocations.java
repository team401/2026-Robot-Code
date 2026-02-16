package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.AllianceUtil;

/**
 * The FieldLocations class provides static methods to get field locations for the current alliance.
 *
 * <p>This is similar to FieldConstants in that it uses static methods to derive locations using
 * data loaded from JSON. However, FieldConstants are the physical constraints of the field, while
 * FieldLocations are 401's user-defined locations, such as target locations for autonomous driving
 * or passing.
 */
public class FieldLocations {
  private FieldLocations() {}

  /**
   * Get the target location for passing to the left side of the field, from the current alliance
   * station's perspective.
   *
   * @return A Translation2d representing the left target passing location for the current alliance
   */
  public static Translation3d leftPassingTarget() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.leftPassingTarget
        : JsonConstants.blueFieldLocations.leftPassingTarget;
  }

  /**
   * Get the target location for passing to the right side of the field, from the current alliance
   * station's perspective.
   *
   * @return A Translation2d representing the right target passing location for the current alliance
   */
  public static Translation3d rightPassingTarget() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.rightPassingTarget
        : JsonConstants.blueFieldLocations.rightPassingTarget;
  }

  public static Pose2d leftClimbLocation() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.leftClimbLocation
        : JsonConstants.blueFieldLocations.leftClimbLocation;
  }

  public static Pose2d rightClimbLocation() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.rightClimbLocation
        : JsonConstants.blueFieldLocations.rightClimbLocation;
  }
}
