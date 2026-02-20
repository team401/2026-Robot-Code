package frc.robot.auto.drive;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;
import frc.robot.util.PIDGains;

public class AutoPilotAction extends AutoAction {

  public APTarget target = null;
  public APProfile profile = null;
  public APConstraints constraints = null;
  public PIDGains pidGains = null;

  @Override
  public Command toCommand(AutoActionData data) {
    if (target == null) {
      return null;
    }

    if (target.getReference() == null) {
      return null;
    }

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

    var headingController = pidGains.toPIDController();

    return DriveCoordinatorCommands.autoPilotCommand(
        data.driveCoordinator(), new Autopilot(profile), target, headingController);
  }
}
