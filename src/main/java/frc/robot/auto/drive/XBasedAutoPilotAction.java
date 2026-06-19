package frc.robot.auto.drive;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import coppercore.wpilib_interface.tuning.PIDGains;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;

public class XBasedAutoPilotAction extends AutoPilotAction {

  /** No-arg constructor retained for JSON deserialization. */
  public XBasedAutoPilotAction() {}

  /** Authoring constructor used by the Java auto generator. */
  public XBasedAutoPilotAction(
      APTarget target,
      APConstraints constraints,
      APProfile profile,
      PIDGains pidGains,
      boolean canMirror) {
    super(target, constraints, profile, pidGains, canMirror);
  }

  @Override
  public Command toCommand(AutoActionContext context) {
    var realTarget = prepareAutoAction(context);

    return DriveCoordinatorCommands.wrapCommand(
        context.driveCoordinator(),
        new DriveCoordinatorCommands.XBasedAutoPilotCommand(
            context.driveCoordinator(),
            new Autopilot(profile),
            realTarget,
            getHeadingController()));
  }
}
