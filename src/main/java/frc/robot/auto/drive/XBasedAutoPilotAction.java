package frc.robot.auto.drive;

import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;

public class XBasedAutoPilotAction extends AutoPilotAction {

  @Override
  public Command toCommand(AutoActionContext context) {
    var realTarget = prepareAutoAction(context);

    return wrapCommand(
        context,
        new DriveCoordinatorCommands.XBasedAutoPilotCommand(
            context.driveCoordinator(),
            new Autopilot(profile),
            realTarget,
            getHeadingController()));
  }
}
