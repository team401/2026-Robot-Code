package frc.robot.auto.drive;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import coppercore.parameter_tools.json.annotations.JSONExclude;
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

  public APTarget target = null;

  @JSONExclude
  APTarget fixedTarget =
      null; // This is the target after alliance-relative transformation is applied

  @GeneratedOptional public APProfile profile = null;
  @GeneratedOptional public APConstraints constraints = null;
  @GeneratedOptional public PIDGains pidGains = null;

  

  protected void handleNullValues(AutoActionContext context) {
    Objects.requireNonNull(target, "Target cannot be null for AutoPilotAction");
    Objects.requireNonNull(
        target.getReference(), "Target Pose cannot be null for AutoPilot Action");

    fixedTarget = target;

    if (context.flipped()) {
        fixedTarget = fixedTarget
            .withReference(Autos.flipPose2d(target.getReference()));
        if (fixedTarget.getEntryAngle().isPresent()){
            fixedTarget = fixedTarget
                .withEntryAngle(Autos.flipRotation2d(target.getEntryAngle().get()));
        }
    }

    if (context.mirrored()) {
        fixedTarget = fixedTarget
            .withReference(Autos.mirrorPose2d(target.getReference()));
        if (fixedTarget.getEntryAngle().isPresent()){
            fixedTarget = fixedTarget
                .withEntryAngle(Autos.mirrorRotation2d(target.getEntryAngle().get()));
        }
    }



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

  @Override
  public Command toCommand(AutoActionContext data) {
    handleNullValues(data);
    var headingController = getHeadingController();
    return wrapCommand(
        data,
        DriveCoordinatorCommands.autoPilotCommand(
            data.driveCoordinator(), new Autopilot(profile), fixedTarget, headingController));
  }
}
