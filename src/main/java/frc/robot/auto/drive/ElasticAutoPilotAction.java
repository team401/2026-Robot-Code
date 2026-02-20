package frc.robot.auto.drive;

import java.util.Objects;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;
import frc.robot.auto.AutoParameters;
import frc.robot.auto.AutoParameters.AutoParameter;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;
import frc.robot.util.PIDGains;

// This is more so an example of how to set up an auto action that can be configured from the dashboard, and less of a specific action we actually want to use.
// and it is not finished yet.
public class ElasticAutoPilotAction extends AutoAction {

  public APTarget target = null;
  public String targetName = null;
  public APProfile profile = null;
  public APConstraints constraints = null;
  public PIDGains pidGains = null;

  public static class APTargetParameter extends AutoParameters.AutoParameter<APTarget> {
    public APTargetParameter(String name, APTarget defaultValue) {
      super(name, defaultValue);
    }

    @Override
    public APTarget getValue() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'getValue'");
    }

    @Override
    public void publishToDashboard() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'publishToDashboard'");
    }
  }

  @Override
  public void setupParameters(AutoActionData data) {
    Objects.requireNonNull(targetName, "Target Name must not be null");
    var auto = data.auto();
    Objects.requireNonNull(auto, "Auto must not be null");

    if (target == null) {
      target = new APTarget(new Pose2d());
    }

    auto.registerParameter(
      new APTargetParameter(targetName, target)
    );
  }

  @Override
  public Command toCommand(AutoActionData data) {
    AutoParameter<?> parameter = data.auto().getParameter(targetName);
    if (parameter == null) {
      throw new IllegalStateException("Parameter with name " + targetName + " not found");
    }
    target = (APTarget) parameter.getValue();

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
