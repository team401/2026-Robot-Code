/*
 * This should go to coppercore geometry, which currently
 * lacks usable classes (and doesn't have unit tests for the ones
 * it does have.)
 */
package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public class Rectangle {
  protected final Translation2d bottomLeft;
  protected final double width;
  protected final double height;
  private Translation2d[] outline;

  public Rectangle(Translation2d bottomLeft, double width, double height) {
    this.bottomLeft = bottomLeft;
    this.width = width;
    this.height = height;
    this.outline =
        new Translation2d[] {
          bottomLeft,
          bottomLeft.plus(new Translation2d(width, 0)),
          bottomLeft.plus(new Translation2d(width, height)),
          bottomLeft.plus(new Translation2d(0, height)),
          bottomLeft,
        };
  }

  public static Rectangle fromCenter(Translation2d center, double width, double height) {
    return new Rectangle(center.minus(new Translation2d(width / 2, height / 2)), width, height);
  }

  /*
   * @returns true if point is inside the rectangle
   */
  public boolean contains(Translation2d point) {
    return bottomLeft.getX() <= point.getX()
        && point.getX() <= bottomLeft.getX() + width
        && bottomLeft.getY() <= point.getY()
        && point.getY() <= bottomLeft.getY() + height;
  }

  public void logAsTrajectory(String path) {
    Logger.recordOutput(path, outline);
  }
}
