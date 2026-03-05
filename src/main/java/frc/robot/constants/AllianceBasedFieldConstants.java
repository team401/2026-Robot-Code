package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.AllianceUtil;

/**
 * The AllianceBasedFieldConstants class provides methods for getting relevant field locations from
 * FieldConstants.java based on which alliance we're on.
 */
public class AllianceBasedFieldConstants {
  public static final Translation3d hubInnerCenterPoint() {
    return AllianceUtil.isRed()
        ? FieldConstants.Hub.oppInnerCenterPoint()
        : FieldConstants.Hub.innerCenterPoint();
  }
}
