package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;

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
  public boolean checkBatteryVoltage(double voltage) {
    return filter.calculate(voltage) < lowVoltageThreshold && DriverStation.isDisabled();
  }
}
