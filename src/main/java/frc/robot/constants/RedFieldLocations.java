package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Contains "field locations" (e.g. passing targets, locations for autonomous or semi-autonomous
 * driving, etc.) for the red alliance
 */
public class RedFieldLocations {
  // TODO: Find actual passing goal locations
  /**
   * Translation to aim for when passing to the left side of the field (from red driver station
   * perspective)
   */
  public final Translation3d leftPassingTarget = new Translation3d();

  /**
   * Translation to aim for when passing to the right side of the field (from red driver station
   * perspective)
   */
  public final Translation3d rightPassingTarget = new Translation3d();
}
