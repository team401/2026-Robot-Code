package frc.robot.subsystems.drive.control_methods;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.drive.AdjustableLinearPath;
import org.littletonrobotics.junction.Logger;

public class LinearDrive extends DriveControlMethod {

  public static class LinearDriveCommand {
    public AdjustableLinearPath.State goalState;
    public LinearDriveProfileConfig profileConfig;

    public LinearDriveCommand(Pose2d goalPose) {
      this(goalPose, new ChassisSpeeds(), LinearDriveProfileConfig.fromJSON());
    }

    public LinearDriveCommand(
        Pose2d goalPose,
        ChassisSpeeds goalEndSpeedFieldRelative,
        LinearDriveProfileConfig profileConfig) {
      this.goalState = new AdjustableLinearPath.State(goalPose, goalEndSpeedFieldRelative);
      this.profileConfig = profileConfig;
    }
  }

  public static class LinearDriveProfileConfig {
    public TrapezoidProfile.Constraints linearConstraints;
    public TrapezoidProfile.Constraints angularConstraints;

    public static LinearDriveProfileConfig fromJSON() {
      return new LinearDriveProfileConfig(
          new TrapezoidProfile.Constraints(
              JsonConstants.driveConstants.linearDriveProfileMaxLinearVelocity.in(MetersPerSecond),
              JsonConstants.driveConstants.linearDriveProfileMaxLinearAcceleration.in(
                  MetersPerSecondPerSecond)),
          new TrapezoidProfile.Constraints(
              JsonConstants.driveConstants.linearDriveProfileMaxAngularVelocity.in(
                  RadiansPerSecond),
              JsonConstants.driveConstants.linearDriveProfileMaxAngularAcceleration.in(
                  RadiansPerSecondPerSecond)));
    }

    public LinearDriveProfileConfig(
        TrapezoidProfile.Constraints linearConstraints,
        TrapezoidProfile.Constraints angularConstraints) {
      this.linearConstraints = linearConstraints;
      this.angularConstraints = angularConstraints;
    }
  }

  protected LinearDriveCommand command;
  protected AdjustableLinearPath linearPath;

  public LinearDrive(Drive drive) {
    super(drive, "LinearDrive", true);
    this.command = null;
    this.linearPath =
        new AdjustableLinearPath(
            new TrapezoidProfile.Constraints(0, 0), new TrapezoidProfile.Constraints(0, 0));
  }

  /**
   * Set the command for this control method. This should be called before activating the control
   * method. Set to null to stop the robot from moving.
   *
   * @param command The command to set for this control method.
   */
  public void setCommand(LinearDriveCommand command) {
    if (this.command == command) {
      return; // No change in command, so do nothing
    }
    this.command = command;
    if (command != null) {
      this.linearPath.setConstraints(
          command.profileConfig.linearConstraints, command.profileConfig.angularConstraints);
    }
  }

  public void clearCommand() {
    setCommand(null);
  }

  public boolean hasCommand() {
    return command != null;
  }

  @Override
  protected void _periodic() {
    if (command == null) {
      return;
    }

    AdjustableLinearPath.State pathState =
        linearPath.calculate(
            JsonConstants.robotInfo.robotPeriod.in(Seconds),
            new AdjustableLinearPath.State(
                drive.getPose(),
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    drive.getChassisSpeeds(), drive.getPose().getRotation())),
            command.goalState);

    drive.setGoalSpeedsBlueOrigins(pathState.speeds);

    Logger.recordOutput(
        "DriveCoordinator/DriveMethods/LinearDrive/goalOmegaRadPerSec",
        pathState.speeds.omegaRadiansPerSecond);
    Logger.recordOutput(
        "DriveCoordinator/DriveMethods/LinearDrive/actualOmegaRadPerSec",
        drive.getChassisSpeeds().omegaRadiansPerSecond);

    Logger.recordOutput(
        "DriveCoordinator/DriveMethods/LinearDrive/Command/GoalPose", command.goalState.pose);
    Logger.recordOutput(
        "DriveCoordinator/DriveMethods/LinearDrive/Command/GoalSpeeds", command.goalState.speeds);
  }

  public boolean isFinished() {
    if (command == null) {
      return true;
    }

    var currentDistanceToTarget = getPoseDistance(drive.getPose(), command.goalState.pose);
    if (!currentDistanceToTarget.isNear(
        Meters.zero(), JsonConstants.driveConstants.linearDriveMaxPositionError)) {
      return false;
    }

    var currentAngularError =
        Radians.of(
            drive.getPose().getRotation().minus(command.goalState.pose.getRotation()).getRadians());
    if (!currentAngularError.isNear(
        Radians.zero(), JsonConstants.driveConstants.linearDriveMaxAngularError)) {
      return false;
    }

    var driveChassisSpeeds = drive.getChassisSpeeds();

    var currentLinearVelocity =
        MetersPerSecond.of(
            Math.hypot(driveChassisSpeeds.vxMetersPerSecond, driveChassisSpeeds.vyMetersPerSecond));
    // Because the goal speeds are field-relative we have to convert them to robot-relative before
    // comparing them to the current robot-relative speeds
    var goalLinearVelocity =
        getChassisLinearVelocity(command.goalState.speeds, command.goalState.pose.getRotation());
    if (!currentLinearVelocity.isNear(
        goalLinearVelocity, JsonConstants.driveConstants.linearDriveMaxLinearVelocityError)) {
      return false;
    }

    var currentAngularVelocity = RadiansPerSecond.of(driveChassisSpeeds.omegaRadiansPerSecond);
    var goalAngularVelocity = RadiansPerSecond.of(command.goalState.speeds.omegaRadiansPerSecond);
    if (!currentAngularVelocity.isNear(
        goalAngularVelocity, JsonConstants.driveConstants.linearDriveMaxAngularVelocityError)) {
      return false;
    }

    return true;
  }

  private static LinearVelocity getChassisLinearVelocity(ChassisSpeeds speeds, Rotation2d heading) {
    // vel = <vx, vy> ⋅ <cos(heading), sin(heading)>
    // vel = vx * cos(heading) + vy * sin(heading)
    return MetersPerSecond.of(
        speeds.vxMetersPerSecond * heading.getCos() + speeds.vyMetersPerSecond * heading.getSin());
  }

  private static Distance getPoseDistance(Pose2d current, Pose2d target) {
    return Meters.of(current.getTranslation().getDistance(target.getTranslation()));
  }
}
