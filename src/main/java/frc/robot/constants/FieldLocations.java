package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
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
   * @return A Translation2d representing the left alliance zone target passing location for the
   *     current alliance
   */
  public static Translation2d leftAZPassingTarget() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.leftAZPassingTarget
        : JsonConstants.blueFieldLocations.leftAZPassingTarget;
  }

  /**
   * Get the target location for passing to the right side of the field, from the current alliance
   * station's perspective.
   *
   * @return A Translation2d representing the right alliance zone target passing location for the
   *     current alliance
   */
  public static Translation2d rightAZPassingTarget() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.rightAZPassingTarget
        : JsonConstants.blueFieldLocations.rightAZPassingTarget;
  }

  /**
   * Get the target location for passing to the left side of the bump, from the current alliance
   * station's perspective.
   *
   * @return A Translation2d representing the left bump target passing location for the current
   *     alliance
   */
  public static Translation2d leftBumpPassingTarget() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.leftBumpPassingTarget
        : JsonConstants.blueFieldLocations.leftBumpPassingTarget;
  }

  /**
   * Get the target location for passing to the right side of the bump, from the current alliance
   * station's perspective.
   *
   * @return A Translation2d representing the right bump target passing location for the current
   *     alliance
   */
  public static Translation2d rightBumpPassingTarget() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.rightBumpPassingTarget
        : JsonConstants.blueFieldLocations.rightBumpPassingTarget;
  }

  /**
   * Get the target location for passing to the left side of the neutral zone, from the current
   * alliance station's perspective.
   *
   * @return A Translation2d representing the left neutral zone target passing location for the
   *     current alliance
   */
  public static Translation2d leftNZPassingTarget() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.leftNZPassingTarget
        : JsonConstants.blueFieldLocations.leftNZPassingTarget;
  }

  /**
   * Get the target location for passing to the right side of the neutral zone, from the current
   * alliance station's perspective.
   *
   * @return A Translation2d representing the right neutral zone target passing location for the
   *     current alliance
   */
  public static Translation2d rightNZPassingTarget() {
    return AllianceUtil.isRed()
        ? JsonConstants.redFieldLocations.rightNZPassingTarget
        : JsonConstants.blueFieldLocations.rightNZPassingTarget;
  }
}
