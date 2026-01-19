package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class DrivetrainConstants {
  public final LinearVelocity maxLinearSpeed = MetersPerSecond.of(0.5);
  public final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(0.5);

  public final LinearVelocity maxLinearDriveLinearSpeed = MetersPerSecond.of(0.5);
  public final AngularVelocity maxLinearDriveAngularSpeed = RadiansPerSecond.of(0.5);

  public final LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(1.0);
  public final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(1.0);

  public final Double joystickDeadband =
      0.06; // Very low deadband for hall effect sticks; needs to be tuned
  public final Double joystickMagnitudeExponent = 2.0;
}
