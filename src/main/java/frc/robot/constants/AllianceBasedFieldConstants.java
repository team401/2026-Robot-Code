package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AllianceUtil.CachedLocation;

/**
 * The AllianceBasedFieldConstants class provides methods for getting relevant field locations from
 * FieldConstants.java based on which alliance we're on.
 */
public class AllianceBasedFieldConstants {
  public static final CachedLocation<Translation3d> hubInnerCenterPoint =
      new CachedLocation<>(
          () ->
              AllianceUtil.isRed()
                  ? FieldConstants.Hub.oppInnerCenterPoint()
                  : FieldConstants.Hub.innerCenterPoint());

  public static final CachedLocation<Translation2d> hubCenterPoint2d =
      new CachedLocation<>(
          () ->
              AllianceUtil.isRed()
                  ? FieldConstants.Hub.oppInnerCenterPoint().toTranslation2d()
                  : FieldConstants.Hub.innerCenterPoint().toTranslation2d());

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
