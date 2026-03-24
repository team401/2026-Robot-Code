package frc.robot.auto.drive;

import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;

public class XBasedAutoPilotAction extends AutoPilotAction {

  @Override
  public Command toCommand(AutoActionContext data) {

    handleNullValues(data);

    return DriveCoordinatorCommands.wrapCommand(
        data.driveCoordinator(),
        new DriveCoordinatorCommands.XBasedAutoPilotCommand(
            data.driveCoordinator(), new Autopilot(profile), fixedTarget, getHeadingController()));
  }
}
