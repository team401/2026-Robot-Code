package frc.robot.auto.coordinationLayer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.AutoAction;

public class StartShooting extends AutoAction {

  @Override
  public Command toCommand(AutoActionContext data) {
    return new InstantCommand(data.coordinationLayer()::startShootingForAuto);
  }
}
