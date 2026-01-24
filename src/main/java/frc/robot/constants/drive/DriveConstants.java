package frc.robot.constants.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.PIDGains;

public class DriveConstants {
  public final LinearVelocity maxLinearSpeed = MetersPerSecond.of(0.5);
  public final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(0.5);

  public final LinearVelocity maxLinearDriveLinearSpeed = MetersPerSecond.of(0.5);
  public final AngularVelocity maxLinearDriveAngularSpeed = RadiansPerSecond.of(0.5);

  public final LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(1.0);
  public final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(1.0);

  public final Double joystickDeadband =
      0.06; // Very low deadband for hall effect sticks; needs to be tuned
  public final Double joystickMagnitudeExponent = 2.0;

  public final PIDGains steerGains = new PIDGains(100, 0.0, 0.5, 0.1, 2.49, 0.0);

  public final PIDGains driveGains = new PIDGains(0.1, 0.0, 0.0, 0.0, 0.124, 0.0);
}
