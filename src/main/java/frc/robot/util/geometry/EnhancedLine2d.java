package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

/**
 * EnhancedLine replaces Line to provide extra functionality that should make its way back to
 * coppercore.
 */
public class EnhancedLine2d {
  // Fields made non-final to support in-place updates via set()
  // [Optimization by Claude Opus 4.5, March 2026]
  protected Translation2d start;
  protected Translation2d end;
  protected Translation2d d;
  protected double length;

  public EnhancedLine2d(Translation2d start, Translation2d end) {
    this.start = start;
    this.end = end;
    this.d = end.minus(start);
    this.length = d.getNorm();
  }

  /**
   * Creates an EnhancedLine2d by projecting two points into the XY plane.
   *
   * @param start Starting point, whose Z will be ignored.
   * @param end Ending point, whose Z will be ignored.
   */
  public EnhancedLine2d(Translation3d start, Translation3d end) {
    this(start.toTranslation2d(), end.toTranslation2d());
  }

  /**
   * Update this line's endpoints in place to avoid allocations.
   *
   * <p>[Optimization by Claude Opus 4.5, March 2026]
   *
   * @param start The new start point
   * @param end The new end point
   */
  public void set(Translation2d start, Translation2d end) {
    this.start = start;
    this.end = end;
    this.d = end.minus(start);
    this.length = d.getNorm();
  }

  private record IntersectionParameters(double s, double t) {}

  /**
   * Intersects this line with line l.
   *
   * <p>Solves: this.start + s * this.d = l.start + t * l.d
   *
   * @param l The line to intersect this with
   * @return An optional containing an IntersectionParameters with s and t if the lines intersect,
   *     empty if they don't.
   */
  private Optional<IntersectionParameters> intersectionParameters(EnhancedLine2d l) {
    var dneg = d.times(-1);

    double D = l.d.cross(dneg);

    // Use Cramer’s rule
    if (D == 0.0) {
      return Optional.empty();
    }

    Translation2d rp = start.minus(l.start);
    return Optional.of(new IntersectionParameters(l.d.cross(rp) / D, rp.cross(dneg) / D));
  }

  public boolean intersects(EnhancedLine2d other) {
    return intersectionParameters(other)
        .map(params -> params.s >= 0.0 && params.s <= 1.0 && params.t >= 0.0 && params.t <= 1.0)
        .orElse(false);
  }
}
