package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
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

  public static PIDController createDefaultAutoPilotHeadingController() {
    var controller = JsonConstants.driveConstants.defaultAutoPilotHeadingGains.toPIDController();
    controller.enableContinuousInput(-Math.PI, Math.PI);
    return controller;
  }

  public static class AutoPilotCommand extends DriveCoordinatorCommands {
    protected Autopilot autoPilot;
    protected APTarget target;
    protected PIDController headingController;

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

      Logger.recordOutput("DriveCoordinator/AutoPilot/goalPose", target.getReference());
    }

    @Override
    public boolean isFinished() {
      var currentPose = driveCoordinator.drive.getPose();
      return autoPilot.atTarget(currentPose, target);
    }
  }

  public static class XBasedAutoPilotCommand extends AutoPilotCommand {
    protected final double directionOfTravelSign;

    public XBasedAutoPilotCommand(
        DriveCoordinator driveCoordinator,
        Autopilot autoPilot,
        APTarget target,
        PIDController headingController) {
      super(driveCoordinator, autoPilot, target, headingController);

      double x = driveCoordinator.drive.getPose().getX();

      directionOfTravelSign = x < target.getReference().getX() ? 1.0 : -1.0;
    }

    @Override
    public boolean isFinished() {
      return super.isFinished()
          || driveCoordinator.drive.getPose().getX() * directionOfTravelSign
              > target.getReference().getX() * directionOfTravelSign;
    }
  }

  public static APConstraints createDefaltAPConstraints() {
    return new APConstraints().withAcceleration(3).withJerk(3);
  }

  public static APProfile createDefaultAPProfile() {
    return new APProfile(createDefaltAPConstraints())
        .withErrorTheta(Degrees.of(0.05))
        .withErrorXY(Meters.of(0.01));
  }

  public static Command joystickDrive(DriveCoordinator driveCoordinator) {
    return driveCoordinator.joystickCommand;
  }

  public static Command stopDrive(DriveCoordinator driveCoordinator) {
    return new StopDriveCommand(driveCoordinator);
  }

  public static Command autoPilotToPoseCommand(
      DriveCoordinator driveCoordinator, Pose2d targetPose) {
    return autoPilotCommand(driveCoordinator, new APTarget(targetPose));
  }

  public static Command autoPilotCommand(DriveCoordinator driveCoordinator, APTarget target) {
    Autopilot autoPilot = new Autopilot(createDefaultAPProfile());
    return autoPilotCommand(driveCoordinator, autoPilot, target);
  }

  public static Command autoPilotToSetPointsCommand(
      DriveCoordinator driveCoordinator, APProfile profile, Pose2d... setpoints) {
    APTarget[] targets = new APTarget[setpoints.length];
    for (int i = 0; i < setpoints.length; i++) {
      targets[i] = new APTarget(setpoints[i]);
    }
    return autoPilotToTargetsCommand(driveCoordinator, profile, targets);
  }

  public static Command autoPilotToTargetsCommand(
      DriveCoordinator driveCoordinator, APProfile profile, APTarget... targets) {
    if (targets.length == 0) {
      return null;
    }
    Autopilot autoPilot = new Autopilot(profile);
    var currentCommand = autoPilotCommand(driveCoordinator, autoPilot, targets[0]);
    for (int i = 1; i < targets.length; i++) {
      currentCommand =
          currentCommand.andThen(autoPilotCommand(driveCoordinator, autoPilot, targets[i]));
    }
    return currentCommand;
  }

  public static Command autoPilotCommand(
      DriveCoordinator driveCoordinator, Autopilot autoPilot, APTarget target) {
    return new AutoPilotCommand(
        driveCoordinator, autoPilot, target, createDefaultAutoPilotHeadingController());
  }

  public static Command autoPilotCommand(
      DriveCoordinator driveCoordinator,
      Autopilot autoPilot,
      APTarget target,
      PIDController headingController) {
    return new AutoPilotCommand(driveCoordinator, autoPilot, target, headingController);
  }
}
