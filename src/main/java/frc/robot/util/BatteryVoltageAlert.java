package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;

/**
 * BatteryVoltageAlert is a utility class that checks whether the battery voltage drops below a
 * given threshold. Internally, it uses a linear filter to see if the battery's voltage is below the
 * given threshold for at least the given filter time.
 */
public class BatteryVoltageAlert {
  private final double lowVoltageThreshold;
  private LinearFilter filter;

  private BatteryVoltageAlert(double lowVoltageThreshold, double filterTime) {
    this.lowVoltageThreshold = lowVoltageThreshold;
    this.filter = LinearFilter.singlePoleIIR(filterTime, 0.02);
  }

  public static BatteryVoltageAlert createBatteryVoltageAlert(
      double lowVoltageThreshold, double filterTime) {
    return new BatteryVoltageAlert(lowVoltageThreshold, filterTime);
  }

  /**
   * Uses a linear filter to compute if the battery is below the given voltage threshold.
   *
   * @param voltage battery voltage
   * @return true if the average battery voltage is below the threshold
   */
  public boolean isBatteryBelowThreshold(double voltage) {
    return filter.calculate(voltage) < lowVoltageThreshold;
  }
}
