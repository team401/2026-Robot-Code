package frc.robot.auto.drive;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.Autos;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;
import frc.robot.util.PIDGains;
import frc.robot.util.ts.GeneratedOptional;
import java.util.Objects;
import java.util.Optional;

public class AutoPilotAction extends DriveAutoAction {

  private APTarget target = null; // original target in blue field coordinates

  @GeneratedOptional public APProfile profile = null;
  @GeneratedOptional public APConstraints constraints = null;
  @GeneratedOptional public PIDGains pidGains = null;

  private void ensureTargetAndPoseAreNotNull() {
    Objects.requireNonNull(target, "Target cannot be null for AutoPilotAction");
    Objects.requireNonNull(
        target.getReference(), "Target Pose cannot be null for AutoPilot Action");
  }

  /*
   * Compute the actual target coordinates and orientation, taking alliance-relative
   * flipping and mirroring parameters into account
   */
  private APTarget computeRealTarget(AutoActionContext context) {
    APTarget realTarget = target;

    if (context.flipped()) {
      realTarget = realTarget.withReference(Autos.flipPose2d(target.getReference()));
      if (realTarget.getEntryAngle().isPresent()) {
        realTarget = realTarget.withEntryAngle(Autos.flipRotation2d(target.getEntryAngle().get()));
      }
    }

    if (context.mirrored() && canMirror) {
      realTarget = realTarget.withReference(Autos.mirrorPose2d(target.getReference()));
      if (realTarget.getEntryAngle().isPresent()) {
        realTarget =
            realTarget.withEntryAngle(Autos.mirrorRotation2d(target.getEntryAngle().get()));
      }
    }
    if (context.mirrored() && !canMirror) {
      System.err.println("Attempt to mirror AutoPilotAction that cannot be mirrored");
    }
    return realTarget;
  }

  /* Replace missing profile and pidgains with default values */
  private void ensureProfileAndPidGains() {
    profile =
        Optional.ofNullable(profile).orElseGet(DriveCoordinatorCommands::createDefaultAPProfile);

    profile =
        profile
            .withConstraints(
                Optional.ofNullable(constraints)
                    .orElse(
                        Optional.ofNullable(profile.getConstraints())
                            .orElseGet(DriveCoordinatorCommands::createDefaultAPConstraints)))
            .withBeelineRadius(
                Optional.ofNullable(profile.getBeelineRadius())
                    .orElse(JsonConstants.driveConstants.defaultAutoPilotBeelineRadius))
            .withErrorTheta(
                Optional.ofNullable(profile.getErrorTheta())
                    .orElse(JsonConstants.driveConstants.defaultAutoPilotHeadingTolerance))
            .withErrorXY(
                Optional.ofNullable(profile.getErrorXY())
                    .orElse(JsonConstants.driveConstants.defaultAutoPilotXYTolerance));

    pidGains =
        Optional.ofNullable(pidGains)
            .orElse(JsonConstants.driveConstants.defaultAutoPilotHeadingGains);
  }

  protected PIDController getHeadingController() {
    var headingController = pidGains.toPIDController();
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    return headingController;
  }

  /* Based on context, prepare this auto action for conversion into a command.s */
  protected APTarget prepareAutoAction(AutoActionContext context) {
    ensureTargetAndPoseAreNotNull();
    var realTarget = computeRealTarget(context);
    ensureProfileAndPidGains();
    return realTarget;
  }

  @Override
  public Command toCommand(AutoActionContext context) {
    var realTarget = prepareAutoAction(context);
    var headingController = getHeadingController();
    return wrapCommand(
        context,
        DriveCoordinatorCommands.autoPilotCommand(
            context.driveCoordinator(), new Autopilot(profile), realTarget, headingController));
  }
}
