package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

// TODO: Move to coppercore

/** The AngleUtil class contains utility methods for dealing with angles */
public class AngleUtil {
  private AngleUtil() {}

  public static final double TAU = Math.PI * 2;

  /**
   * Given an angle in radians, normalize it to be within the range of [0, 2pi).
   *
   * <p>For example, an angle of -pi would become pi while an angle of 3pi/2 would remain unchanged.
   *
   * @param originalHeadingRadians A double representing the heading to normalize, in radians
   * @return A double representing a coterminal angle of originalHeadingRadians (having the same
   *     heading) but satisfying the condition 0 <= angle < 2pi
   */
  public static double normalizeHeading(double originalHeadingRadians) {
    // First, mod by 2pi, placing any angle into the range -2pi to 2pi
    // Next, add 2pi, so that all angles will be from 0 to 2pi
    // Finally, mod by 2pi again, reducing all angles to be 0 to 2pi
    return ((originalHeadingRadians % TAU) + TAU) % TAU;
  }

  /**
   * Given heading in a Rotation2d, normalize it to be within the range of [0, 2pi).
   *
   * <p>For example, an heading of -pi would become pi while a heading of 3pi/2 would remain
   * unchanged.
   *
   * <p>This is useful for converting from the heading that an odometry system might give you to a
   * heading that can be used in a system whose minimum angle is zero, like a turret.
   *
   * @param originalHeading A Rotation2d containing the original heading
   * @return A Rotation2d containing a coterminal angle of originalHeading (having the same heading)
   *     but satisfying the condition 0 <= angle < 2pi
   */
  public static Rotation2d normalizeHeading(Rotation2d originalHeading) {
    return new Rotation2d(normalizeHeading(originalHeading.getRadians()));
  }
}
