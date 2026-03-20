package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;

public class AutoReference extends AutoAction {

  public String name;

  @Override
  public Command toCommand(AutoActionContext data) {
    return data.autos().getRoutineCommandReference(name);
  }
}
