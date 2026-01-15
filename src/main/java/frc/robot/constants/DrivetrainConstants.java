package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class DrivetrainConstants {
  public final LinearVelocity maxLinearSpeed = MetersPerSecond.of(1.0);
  public final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(1.0);
  public final Double joystickDeadband =
      0.06; // Very low deadband for hall effect sticks; needs to be tuned
  public final Double joystickMagnitudeExponent = 2.0;
}
