package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction.AutoActionData;
import frc.robot.subsystems.drive.DriveCoordinator;

public class Auto {

  private AutoAction auto;

  @JSONExclude public AutoAction.AutoActionData data;


  public void setData(DriveCoordinator driveCoordinator) {
    data = new AutoActionData(driveCoordinator, this);
  }

  public Command toCommand() {
    if (data == null) {
      throw new IllegalStateException("Auto data has not been set. Call setData() first.");
    }
    return auto.toCommand(data);
  }

}
