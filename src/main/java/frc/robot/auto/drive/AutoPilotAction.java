package frc.robot.auto.drive;

import java.util.Objects;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;
import frc.robot.util.PIDGains;
import frc.robot.util.ts.TypeScriptOptional;

public class AutoPilotAction extends AutoAction {

  public APTarget target = null;
  @TypeScriptOptional public APProfile profile = null;
  @TypeScriptOptional public APConstraints constraints = null;
  @TypeScriptOptional public PIDGains pidGains = null;

  @Override
  public Command toCommand(AutoActionData data) {
    Objects.requireNonNull(target, "Target cannot be null for AutoPilotAction");
    Objects.requireNonNull(target.getReference(), "Target Pose cannot be null for AutoPilot Action");

    if (profile == null) {
      profile = DriveCoordinatorCommands.createDefaultAPProfile();
      if (constraints != null) {
        profile = profile.withConstraints(constraints);
      }
    } else {
      if (constraints != null) {
        profile = profile.withConstraints(constraints);
      }
      if (profile.getBeelineRadius() == null) {
        profile =
            profile.withBeelineRadius(JsonConstants.driveConstants.defaultAutoPilotBeelineRadius);
      }
      if (profile.getErrorTheta() == null) {
        profile =
            profile.withErrorTheta(JsonConstants.driveConstants.defaultAutoPilotHeadingTolerance);
      }
      if (profile.getErrorXY() == null) {
        profile = profile.withErrorXY(JsonConstants.driveConstants.defaultAutoPilotXYTolerance);
      }
    }

    PIDGains gains = pidGains;

    if (gains == null) {
      gains = JsonConstants.driveConstants.defaultAutoPilotHeadingGains;
    }

    var headingController = gains.toPIDController();

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return DriveCoordinatorCommands.autoPilotCommand(
        data.driveCoordinator(), new Autopilot(profile), target, headingController);
  }
}
