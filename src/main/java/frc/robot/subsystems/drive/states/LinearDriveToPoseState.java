package frc.robot.subsystems.drive.states;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.utility.LinearPath;
import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

// Copilot used to help write this class

/**
 * State that drives the robot linearly toward a desired 2D pose (translation + rotation).
 *
 * <p>Responsibilities:
 * - Accepts a target {@code Pose2d} (via {@code setTargetPose}) and drives the robot toward that
 *   pose using a {@link LinearPath} motion profile for combined linear and angular motion.
 * - Rebuilds the {@link LinearPath} whenever linear or angular trapezoidal profile constraints are
 *   updated via {@code setLinearConstraints}, {@code setAngularConstraints}, or
 *   {@code setConstraints}.
 * - On each periodic update, computes the next goal chassis speeds from the {@link LinearPath},
 *   applies them to the drive subsystem via {@code world.setGoalSpeedsBlueOrigins(...)}, and
 *   records selected diagnostic values to {@code Logger}.
 * - Monitors pose, velocity, and angular error and finishes the state when all errors are within
 *   configurable tolerances.
 *
 * <p>Key behavior and contracts:
 * - The class extends {@code State<Drive>} and is intended to be used inside a drive state
 *   machine. The constructor initializes default trapezoidal-profile constraints from
 *   {@code JsonConstants.driveConstants.linearDriveProfile*}.
 * - Constraints are expressed in SI units:
 *   - Linear velocities/accelerations: meters / seconds and meters / seconds^2.
 *   - Angular velocities/accelerations: radians / seconds and radians / seconds^2.
 * - The periodic update uses the configured robot period value obtained from
 *   {@code JsonConstants.robotInfo.robotPeriod.in(Seconds)} when invoking
 *   {@code LinearPath.calculate(...)}.
 * - The state records the following Logger keys (example):
 *   - "Drive/LinearDriveToPoseState/goalOmegaRadPerSec"
 *   - "Drive/LinearDriveToPoseState/actualOmegaRadPerSec"
 *
 * <p>Finish condition:
 * - The state calls {@code finish()} when all of the following are true:
 *   - Current heading is within {@code JsonConstants.driveConstants.linearDriveMaxAngularError} of
 *     the target heading.
 *   - Position error is within {@code JsonConstants.driveConstants.linearDriveMaxPositionError}.
 *   - Linear speed is within {@code JsonConstants.driveConstants.linearDriveMaxLinearVelocityError}
 *     of zero.
 *   - Angular velocity is within {@code JsonConstants.driveConstants.linearDriveMaxAngularVelocityError}
 *     of zero.
 *
 * <p>Usage notes:
 * - Clients must provide a non-null target pose via {@code setTargetPose} before relying on the
 *   state to drive to a location.
 * - Constraints may be adjusted at runtime; changing constraints causes a new {@link LinearPath}
 *   instance to be constructed so that subsequent periodic calls use the updated profile.
 *
 * <p>Limitations and thread-safety:
 * - This class is not explicitly synchronized; callers should ensure that configuration methods
 *   ({@code setTargetPose}, {@code set*Constraints}) are not racing with periodic execution from a
 *   different thread, or provide external synchronization if required.
 */
public class LinearDriveToPoseState extends State<Drive> {

  private Pose2d targetPose;

  private TrapezoidProfile.Constraints linearConstraints;
  private TrapezoidProfile.Constraints angularConstraints;

  private LinearPath linearPath;

  public void setTargetPose(Pose2d targetPose) {
    this.targetPose = targetPose;
  }

  protected void setLinearConstraints(TrapezoidProfile.Constraints constraints) {
    this.linearConstraints = constraints;
    reconstructLinearPath();
  }

  protected void setAngularConstraints(TrapezoidProfile.Constraints constraints) {
    this.angularConstraints = constraints;
    reconstructLinearPath();
  }

  protected void setConstraints(
      TrapezoidProfile.Constraints linearConstraints,
      TrapezoidProfile.Constraints angularConstraints) {
    this.linearConstraints = linearConstraints;
    this.angularConstraints = angularConstraints;
    reconstructLinearPath();
  }

  protected void reconstructLinearPath() {
    linearPath = new LinearPath(linearConstraints, angularConstraints);
  }

  public LinearDriveToPoseState() {
    super("LinearDrive");

    setConstraints(
        new TrapezoidProfile.Constraints(
            JsonConstants.driveConstants.linearDriveProfileMaxLinearVelocity.in(MetersPerSecond),
            JsonConstants.driveConstants.linearDriveProfileMaxLinearAcceleration.in(
                MetersPerSecondPerSecond)),
        new TrapezoidProfile.Constraints(
            JsonConstants.driveConstants.linearDriveProfileMaxAngularVelocity.in(RadiansPerSecond),
            JsonConstants.driveConstants.linearDriveProfileMaxAngularAcceleration.in(
                RadiansPerSecondPerSecond)));
  }


  @Override
  protected void periodic(StateMachine<Drive> stateMachine, Drive world) {

    LinearPath.State pathState =
        linearPath.calculate(
            JsonConstants.robotInfo.robotPeriod.in(Seconds),
            new LinearPath.State(
                world.getPose(),
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    world.getChassisSpeeds(), world.getPose().getRotation())),
            targetPose);

    world.setGoalSpeedsBlueOrigins(pathState.speeds);

    Logger.recordOutput(
        "Drive/LinearDriveToPoseState/goalOmegaRadPerSec", pathState.speeds.omegaRadiansPerSecond);
    Logger.recordOutput(
        "Drive/LinearDriveToPoseState/actualOmegaRadPerSec",
        world.getChassisSpeeds().omegaRadiansPerSecond);

    Distance positionError = getPoseDistance(world.getPose(), targetPose);

    Angle currentRotation = Rotations.of(world.getPose().getRotation().getRotations());
    Angle targetRotation = Rotations.of(targetPose.getRotation().getRotations());

    LinearVelocity currentVelocity =
        MetersPerSecond.of(
            Math.hypot(
                world.getChassisSpeeds().vxMetersPerSecond,
                world.getChassisSpeeds().vyMetersPerSecond));
    AngularVelocity currentAngularVelocity =
        RadiansPerSecond.of(world.getChassisSpeeds().omegaRadiansPerSecond);

    if (currentRotation.isNear(
            targetRotation, JsonConstants.driveConstants.linearDriveMaxAngularError)
        && positionError.isNear(
            Meters.zero(), JsonConstants.driveConstants.linearDriveMaxPositionError)
        && currentVelocity.isNear(
            MetersPerSecond.zero(), JsonConstants.driveConstants.linearDriveMaxLinearVelocityError)
        && currentAngularVelocity.isNear(
            RadiansPerSecond.zero(),
            JsonConstants.driveConstants.linearDriveMaxAngularVelocityError)) {
      finish();
      return;
    }
  }

  private static Distance getPoseDistance(Pose2d current, Pose2d target) {
    return Meters.of(current.getTranslation().getDistance(target.getTranslation()));
  }
}
