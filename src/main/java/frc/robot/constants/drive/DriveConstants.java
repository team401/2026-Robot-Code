package frc.robot.constants.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.PIDGains;

/**
 * Constants for the drivetrain subsystem related to driving performance and control.
 *
 * <p>These include maximum speeds, profile constraints for driving to a pose, tolerances for
 * considering the robot "at" a target pose, joystick deadband and response shaping, and PID gains
 * for steering and driving.
 *
 * <p>Values related to physical dimensions or motor configurations are located in {@link
 * PhysicalDriveConstants}.
 */
public class DriveConstants {
  public final LinearVelocity maxLinearSpeed = MetersPerSecond.of(4.0);
  public final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(7.2);

  public final Double joystickDeadband =
      0.02; // Very low deadband for hall effect sticks; needs to be tuned
  public final Double joystickMagnitudeExponent = 2.0;

  // Linear Drive to Pose Profile Constraints

  public final LinearVelocity linearDriveProfileMaxLinearVelocity = MetersPerSecond.of(4.0);
  public final AngularVelocity linearDriveProfileMaxAngularVelocity = RadiansPerSecond.of(6.2);

  public final LinearAcceleration linearDriveProfileMaxLinearAcceleration =
      MetersPerSecondPerSecond.of(6.0);
  public final AngularAcceleration linearDriveProfileMaxAngularAcceleration =
      RadiansPerSecondPerSecond.of(6.0);

  // Tolerances for linear driving to a pose
  // This is the maximum allowable error in rotation to consider the robot "at" the target
  public final Angle linearDriveMaxAngularError = Degrees.of(5.0);
  // This is the maximum allowable error in position to consider the robot "at" the target
  public final Distance linearDriveMaxPositionError = Meters.of(0.05);
  // This is the maximum allowable linear speed to consider the robot "at" the target
  public final LinearVelocity linearDriveMaxLinearVelocityError = MetersPerSecond.of(0.05);
  // This is the maximum allowable angular speed to consider the robot "at" the target
  public final AngularVelocity linearDriveMaxAngularVelocityError = RadiansPerSecond.of(0.05);

  public PIDGains steerGains = new PIDGains(100, 0.0, 0.5, 0.1, 0.0, 2.49, 0.0);

  public PIDGains driveGains = new PIDGains(0.1, 0.0, 0.0, 0.20845, 0.0, 0.75722, 0.0);

  public PIDGains defaultAutoPilotHeadingGains = new PIDGains(3.0, 0.0, 0.0);
}
