package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

/**
 * The AllianceUtil class provides an easy way to check which alliance we are currently on.
 *
 * <p>It assumes red as the default alliance. Be sure to test both alliances before competition to
 * ensure there are no inversion issues.
 */
public class AllianceUtil {
  /**
   * Tracks caching of a field location of unknown type based on what alliance we're on.
   *
   * <p>Call {@link #get()} to get the value.
   *
   * <p>By checking for which alliance the cache was generated, it can be updated for different
   * alliances without restarting code
   */
  public static class CachedLocation<T> {
    private Alliance allianceForCache = null;
    private T cachedValue = null;
    private final Supplier<T> initializer;

    public CachedLocation(Supplier<T> initializer) {
      this.initializer = initializer;
    }

    public T get() {
      Alliance currentAlliance = getAlliance();
      if (cachedValue == null || allianceForCache != currentAlliance) {
        cachedValue = initializer.get();
      }

      return cachedValue;
    }
  }

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
    return getAlliance() == Alliance.Red;
  }

  /**
   * Gets the current alliance.
   *
   * <p>If no alliance is present, this method defaults to the Red alliance.
   *
   * @return {@code Blue} if the DriverStation has provided an alliance that alliance is blue,
   *     {@code Red} otherwise.
   */
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  /**
   * Return the opposite alliance than {@link #getAlliance()}.
   *
   * @return {@code Red} if {@link #getAlliance()} returns {@code Blue}, {@code Blue} otherwise
   */
  public static Alliance getOppAlliance() {
    return switch (getAlliance()) {
      case Red -> Alliance.Blue;
      case Blue -> Alliance.Red;
    };
  }
}
