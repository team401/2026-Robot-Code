package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.AllianceUtil;

/**
 * The AllianceBasedFieldConstants class provides methods for getting relevant field locations from
 * FieldConstants.java based on which alliance we're on.
 */
public class AllianceBasedFieldConstants {
  /**
   * Tracks caching of a field location of unknown type based on what alliance we're on.
   *
   * <p>By checking for which alliance the cache was generated, it can be updated for different
   * alliances without restarting code
   */
  private record CachedLocation<T>(Alliance alliance, T value) {}

  private static CachedLocation<Translation3d> hubInnerCenterPoint;

  public static final Translation3d hubInnerCenterPoint() {
    Alliance alliance = AllianceUtil.getAlliance();

    if (hubInnerCenterPoint == null || hubInnerCenterPoint.alliance() != alliance) {
      Translation3d value =
          AllianceUtil.isRed()
              ? FieldConstants.Hub.oppInnerCenterPoint()
              : FieldConstants.Hub.innerCenterPoint();
      hubInnerCenterPoint = new CachedLocation<Translation3d>(AllianceUtil.getAlliance(), value);
    }

    return hubInnerCenterPoint.value();
  }

  private static CachedLocation<Translation2d> hubCenterPoint2d;

  public static final Translation2d hubCenterPoint2d() {
    Alliance alliance = AllianceUtil.getAlliance();

    if (hubCenterPoint2d == null || hubCenterPoint2d.alliance() != alliance) {
      Translation2d value =
          AllianceUtil.isRed()
              ? FieldConstants.Hub.oppInnerCenterPoint().toTranslation2d()
              : FieldConstants.Hub.innerCenterPoint().toTranslation2d();

      hubCenterPoint2d = new CachedLocation<Translation2d>(alliance, value);
    }

    return hubCenterPoint2d.value();
  }

  /**
   * Check whether the given pose is within the alliance zone.
   *
   * @param robotPose A Pose2d containing the current position of the robot to verify
   * @return {@code true} if the robot is within its own alliance zone, {@code false} otherwise.
   */
  public static final boolean isInAllianceZone(Pose2d robotPose) {
    Alliance alliance = AllianceUtil.getAlliance();

    return switch (alliance) {
      case Red -> robotPose.getX() > FieldConstants.LinesVertical.oppAllianceZone();
      case Blue -> robotPose.getX() < FieldConstants.LinesVertical.allianceZone();
    };
  }
}
