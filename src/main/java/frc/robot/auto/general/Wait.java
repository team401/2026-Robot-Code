package frc.robot.auto.general;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoAction;

public class Wait extends AutoAction {
  public Time delay;

  @Override
  public Command toCommand(AutoActionContext data) {
    return new WaitCommand(delay);
  }
}
