package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;

public class BatteryVoltageAlert {
  private final double lowVoltageThreshold;
  private LinearFilter filter = LinearFilter.singlePoleIIR(0.05, 0.02);

  private BatteryVoltageAlert(double lowVoltageThreshold) {
    this.lowVoltageThreshold = lowVoltageThreshold;
    this.filter = LinearFilter.singlePoleIIR(0.05, 0.02);
  }

  public static BatteryVoltageAlert createBatteryVoltageAlert(double lowVoltageThreshold) {
    return new BatteryVoltageAlert(lowVoltageThreshold);
  }

  public boolean checkBatteryVoltage(double voltage) {
    return filter.calculate(voltage) < lowVoltageThreshold && DriverStation.isDisabled();
  }
}
