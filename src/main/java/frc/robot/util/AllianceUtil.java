package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The AllianceUtil class provides an easy way to check which alliance we are currently on.
 *
 * <p>It assumes red as the default alliance. Be sure to test both alliances before competition to
 * ensure there are no inversion issues.
 */
public class AllianceUtil {
  private AllianceUtil() {}

  /**
   * Checks whether the current alliance is Red.
   *
   * <p>If no alliance is present, this method defaults to the Red alliance.
   *
   * @return {@code false} if the DriverStation has provided an alliance that alliance is blue,
   *     {@code true} otherwise.
   */
  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
  }
}
