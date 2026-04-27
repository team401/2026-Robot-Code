package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Contains "field locations" (e.g. passing targets, locations for autonomous or semi-autonomous
 * driving, etc.) for one alliance. These values should not be referenced directly, but should
 * instead be referenced with the methods in FieldLocations.java.
 */
public class FieldLocationInstance {
  /**
   * Translation to aim for when passing to the left side of the alliance zone (from driver station
   * perspective)
   */
  public final Translation2d leftAZPassingTarget = new Translation2d();

  /**
   * Translation to aim for when passing to the right side of the alliance zone (from driver station
   * perspective)
   */
  public final Translation2d rightAZPassingTarget = new Translation2d();

  /**
   * Translation to aim for when passing to the left side of the bump (on the side of the neutral
   * zone, from driver station perspective)
   */
  public final Translation2d leftBumpPassingTarget = new Translation2d();

  /**
   * Translation to aim for when passing to the right side of the bump (on the side of the neutral
   * zone, from driver station perspective)
   */
  public final Translation2d rightBumpPassingTarget = new Translation2d();

  /**
   * Translation to aim for when passing to the left side of the neutral zone (from driver station
   * perspective)
   */
  public final Translation2d leftNZPassingTarget = new Translation2d();

  /**
   * Translation to aim for when passing to the right side of the neutral zone (from driver station
   * perspective)
   */
  public final Translation2d rightNZPassingTarget = new Translation2d();
}
