package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;

public class BatteryVoltageAlert {
    private final static double LOW_VOLTAGE = 12.2;
    private final static LinearFilter filter = LinearFilter.movingAverage(5);

    public static boolean checkBatteryVoltage(double voltage) {
        if (filter.calculate(voltage) < LOW_VOLTAGE && DriverStation.isDisabled()) {
            return true;
        }
        return false;
    }
}
