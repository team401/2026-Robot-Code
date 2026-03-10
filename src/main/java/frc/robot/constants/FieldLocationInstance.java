package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Contains "field locations" (e.g. passing targets, locations for autonomous or semi-autonomous
 * driving, etc.) for one alliance. These values should not be referenced directly, but should
 * instead be referenced with the methods in FieldLocations.java.
 */
public class FieldLocationInstance {
  // TODO: Find actual passing goal locations
  /**
   * Translation to aim for when passing to the left side of the field (from driver station
   * perspective)
   */
  public final Translation2d leftPassingTarget = new Translation2d();

  /**
   * Translation to aim for when passing to the right side of the field (from driver station
   * perspective)
   */
  public final Translation2d rightPassingTarget = new Translation2d();

  public final Pose2d leftClimbLocation = new Pose2d();
  public final Pose2d rightClimbLocation = new Pose2d();
}
