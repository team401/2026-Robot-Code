package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
import frc.robot.util.drive.AdjustableLinearPath;
import org.littletonrobotics.junction.Logger;

public class DriveCoordinatorCommands extends Command {

  // Add logging

  protected DriveCoordinator driveCoordinator;

  public DriveCoordinatorCommands(DriveCoordinator driveCoordinator) {
    this.driveCoordinator = driveCoordinator;
  }

  private static class StopDriveCommand extends DriveCoordinatorCommands {

    public StopDriveCommand(DriveCoordinator driveCoordinator) {
      super(driveCoordinator);
    }

    @Override
    public void execute() {
      driveCoordinator.drive.stop();
    }
  }

  public static record LinearDriveGoal(
      Pose2d targetPose, LinearVelocity endLinearVelocity, AngularVelocity endAngularVelocity) {

    public static LinearDriveGoal toPose(Pose2d targetPose) {
      return new LinearDriveGoal(targetPose, MetersPerSecond.zero(), RadiansPerSecond.zero());
    }

    public static LinearDriveGoal toPoseWithEndVelocities(
        Pose2d targetPose, LinearVelocity endLinearVelocity, AngularVelocity endAngularVelocity) {
      return new LinearDriveGoal(targetPose, endLinearVelocity, endAngularVelocity);
    }
  }

  public static record LinearDriveProfileConfig(
      TrapezoidProfile.Constraints linearConstraints,
      TrapezoidProfile.Constraints angularConstraints,
      boolean linearSpeedCheckEnabled,
      boolean angularSpeedCheckEnabled) {

    public LinearDriveProfileConfig(
        TrapezoidProfile.Constraints linearConstraints,
        TrapezoidProfile.Constraints angularConstraints) {
      this(linearConstraints, angularConstraints, true, true);
    }

    public static LinearDriveProfileConfig fromJSON() {
      return fromJSONWithVelocityChecks(true, true);
    }

    public static LinearDriveProfileConfig fromJSONWithVelocityChecks(
        boolean linearSpeedCheckEnabled, boolean angularSpeedCheckEnabled) {
      return new LinearDriveProfileConfig(
          new TrapezoidProfile.Constraints(
              JsonConstants.driveConstants.linearDriveProfileMaxLinearVelocity.in(MetersPerSecond),
              JsonConstants.driveConstants.linearDriveProfileMaxLinearAcceleration.in(
                  MetersPerSecondPerSecond)),
          new TrapezoidProfile.Constraints(
              JsonConstants.driveConstants.linearDriveProfileMaxAngularVelocity.in(
                  RadiansPerSecond),
              JsonConstants.driveConstants.linearDriveProfileMaxAngularAcceleration.in(
                  RadiansPerSecondPerSecond)),
          linearSpeedCheckEnabled,
          angularSpeedCheckEnabled);
    }
  }

  private static class FollowLinearPathCommand extends DriveCoordinatorCommands {
    private LinearDriveGoal linearDriveGoal;
    protected AdjustableLinearPath linearPath;

    public FollowLinearPathCommand(
        DriveCoordinator driveCoordinator, LinearDriveGoal linearDriveGoal) {
      this(driveCoordinator, linearDriveGoal, LinearDriveProfileConfig.fromJSON());
    }

    public FollowLinearPathCommand(
        DriveCoordinator driveCoordinator,
        LinearDriveGoal linearDriveGoal,
        LinearDriveProfileConfig profileConfig) {
      super(driveCoordinator);
      this.linearDriveGoal = linearDriveGoal;
      this.linearPath =
          new AdjustableLinearPath(
              profileConfig.linearConstraints, profileConfig.angularConstraints);
    }

    @Override
    public void execute() {
      if (linearDriveGoal == null) {
        return;
      }

      AdjustableLinearPath.State pathState =
          linearPath.calculate(
              JsonConstants.robotInfo.robotPeriod.in(Seconds),
              new AdjustableLinearPath.State(
                  driveCoordinator.drive.getPose(),
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      driveCoordinator.drive.getChassisSpeeds(),
                      driveCoordinator.drive.getPose().getRotation())),
              linearDriveGoal.targetPose,
              linearDriveGoal.endLinearVelocity,
              linearDriveGoal.endAngularVelocity);

      driveCoordinator.drive.setGoalSpeedsBlueOrigins(pathState.speeds);

      // TODO: Change logging paths

      Logger.recordOutput("DriveCoordinator/LinearDrive/goalPose", linearDriveGoal.targetPose);
      Logger.recordOutput(
          "DriveCoordinator/LinearDrive/goalOmegaRadPerSec",
          pathState.speeds.omegaRadiansPerSecond);
      Logger.recordOutput(
          "DriveCoordinator/LinearDrive/actualOmegaRadPerSec",
          driveCoordinator.drive.getChassisSpeeds().omegaRadiansPerSecond);
    }

    @Override
    public boolean isFinished() {
      return isWithinPositionTolerance(
              driveCoordinator.drive,
              linearDriveGoal.targetPose,
              JsonConstants.driveConstants.linearDriveMaxPositionError,
              JsonConstants.driveConstants.linearDriveMaxAngularError)
          && isWithinVelocityTolerance(
              driveCoordinator.drive,
              linearDriveGoal.endLinearVelocity,
              linearDriveGoal.endAngularVelocity,
              JsonConstants.driveConstants.linearDriveMaxLinearVelocityError,
              JsonConstants.driveConstants.linearDriveMaxAngularVelocityError);
    }
  }

  public static PIDController createDefaultAutoPilotHeadingController() {
    return JsonConstants.driveConstants.defaultAutoPilotHeadingGains.toPIDController();
  }

  public static class AutoPilotCommand extends DriveCoordinatorCommands {
    private Autopilot autoPilot;
    private APTarget target;
    private PIDController headingController;

    public AutoPilotCommand(
        DriveCoordinator driveCoordinator,
        Autopilot autoPilot,
        APTarget target,
        PIDController headingController) {
      super(driveCoordinator);
      this.autoPilot = autoPilot;
      this.target = target;
      this.headingController = headingController;
    }

    @Override
    public void initialize() {
      headingController.reset();
    }

    @Override
    public void execute() {
      var chassisSpeeds = driveCoordinator.drive.getChassisSpeeds();
      var currentPose = driveCoordinator.drive.getPose();

      APResult output = autoPilot.calculate(currentPose, chassisSpeeds, target);

      // TODO: Figure out how to convert target heading into rotation speed.
      var desiredHeading = output.targetAngle();
      var currentHeading = currentPose.getRotation();

      double omega =
          headingController.calculate(currentHeading.getRadians(), desiredHeading.getRadians());

      var speeds =
          new ChassisSpeeds(
              output.vx().in(MetersPerSecond), output.vy().in(MetersPerSecond), omega);

      driveCoordinator.drive.setGoalSpeedsBlueOrigins(speeds);
    }

    @Override
    public boolean isFinished() {
      var currentPose = driveCoordinator.drive.getPose();
      return autoPilot.atTarget(currentPose, target);
    }
  }

  public static Command linearDriveToPose(DriveCoordinator driveCoordinator, Pose2d targetPose) {
    return new FollowLinearPathCommand(driveCoordinator, LinearDriveGoal.toPose(targetPose));
  }

  public static Command linearDriveToGoal(DriveCoordinator driveCoordinator, LinearDriveGoal goal) {
    return new FollowLinearPathCommand(driveCoordinator, goal);
  }

  public static Command linearDriveWithConfig(
      DriveCoordinator driveCoordinator,
      LinearDriveGoal linearDriveGoal,
      LinearDriveProfileConfig profileConfig) {
    return new FollowLinearPathCommand(driveCoordinator, linearDriveGoal, profileConfig);
  }

  public static Command joystickDrive(DriveCoordinator driveCoordinator) {
    return driveCoordinator.joystickCommand;
  }

  public static Command stopDrive(DriveCoordinator driveCoordinator) {
    return new StopDriveCommand(driveCoordinator);
  }

  public static boolean isWithinPositionTolerance(
      Drive drive, Pose2d targetPose, Distance linearTolerance, Angle angularTolerance) {
    var currentPose = drive.getPose();
    var currentDistanceToTarget =
        Meters.of(currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    if (!currentDistanceToTarget.isNear(Meters.zero(), linearTolerance)) {
      return false;
    }

    var currentAngularError =
        Radians.of(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (!currentAngularError.isNear(Radians.zero(), angularTolerance)) {
      return false;
    }

    return true;
  }

  public static boolean isWithinVelocityTolerance(
      Drive drive,
      LinearVelocity targetLinearVelocity,
      AngularVelocity targetAngularVelocity,
      LinearVelocity linearTolerance,
      AngularVelocity angularTolerance) {
    var chassisSpeeds = drive.getChassisSpeeds();

    var currentLinearVelocity =
        MetersPerSecond.of(
            Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
    if (!currentLinearVelocity.isNear(targetLinearVelocity, linearTolerance)) {
      return false;
    }

    var currentAngularVelocity = RadiansPerSecond.of(chassisSpeeds.omegaRadiansPerSecond);
    if (!currentAngularVelocity.isNear(targetAngularVelocity, angularTolerance)) {
      return false;
    }

    return true;
  }
}
