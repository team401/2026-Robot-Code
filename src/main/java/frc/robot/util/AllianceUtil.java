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
   * Tracks caching of a constant (e.g. a field location) of unknown type based on what alliance
   * we're on.
   *
   * <p>Call {@link #get()} to get the value.
   *
   * <p>This is intended for "constants" that need to be calculated for each alliance but will never
   * change as long as the alliance stays the same.
   *
   * <p>By checking for which alliance the cache was generated, it can be updated for different
   * alliances without restarting code
   */
  public static class AllianceBasedValue<T> {
    private Alliance allianceForCache = null;
    private T cachedValue = null;
    private final Supplier<T> initializer;

    /**
     * Create a new AllianceBasedValue, given a Supplier to create the value when needed
     *
     * @param initializer A Supplier of the value. This will be called whenever the value is
     *     accessed and either the value has not yet been generated or the current alliance has
     *     changed since the value was generated. This supplier must generate the correct value
     *     based on what alliance is currently active. {@link AllianceUtil#getAlliance()} can be
     *     used for this purpose.
     */
    public AllianceBasedValue(Supplier<T> initializer) {
      this.initializer = initializer;
    }

    /**
     * Get the current value. If there isn't a cached value or that cached value was generated for a
     * different alliance than the current alliance, the initializer will be called to generate and
     * cache a new value.
     *
     * @return The current value.
     */
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
