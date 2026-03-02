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
import frc.robot.util.ts.TypeScriptOptional;
import java.util.Objects;
import java.util.Optional;

public class AutoPilotAction extends AutoAction {

  public APTarget target = null;
  @TypeScriptOptional public APProfile profile = null;
  @TypeScriptOptional public APConstraints constraints = null;
  @TypeScriptOptional public PIDGains pidGains = null;

  @Override
  public Command toCommand(AutoActionContext data) {
    Objects.requireNonNull(target, "Target cannot be null for AutoPilotAction");
    Objects.requireNonNull(
        target.getReference(), "Target Pose cannot be null for AutoPilot Action");

    profile = Optional.ofNullable(profile).orElseGet(DriveCoordinatorCommands::createDefaultAPProfile);
    
    profile = profile
      .withConstraints(Optional.ofNullable(constraints)
        .orElse(Optional.ofNullable(profile.getConstraints())
            .orElseGet(DriveCoordinatorCommands::createDefaultAPConstraints)))
      .withBeelineRadius(Optional.ofNullable(profile.getBeelineRadius())
        .orElse(JsonConstants.driveConstants.defaultAutoPilotBeelineRadius))
      .withErrorTheta(Optional.ofNullable(profile.getErrorTheta())
        .orElse(JsonConstants.driveConstants.defaultAutoPilotHeadingTolerance))
      .withErrorXY(Optional.ofNullable(profile.getErrorXY())
        .orElse(JsonConstants.driveConstants.defaultAutoPilotXYTolerance));

    PIDGains gains = Optional.ofNullable(pidGains).orElse(JsonConstants.driveConstants.defaultAutoPilotHeadingGains);

    var headingController = gains.toPIDController();

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return DriveCoordinatorCommands.autoPilotCommand(
        data.driveCoordinator(), new Autopilot(profile), target, headingController);
  }
}
