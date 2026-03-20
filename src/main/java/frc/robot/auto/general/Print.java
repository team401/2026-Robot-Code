package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.AutoAction;

// This will allow us to print messages to the console during auto to help with debugging
public class Print extends AutoAction {
  public String message;

  @Override
  public Command toCommand(AutoActionContext data) {
    return new InstantCommand(
        () -> {
          System.out.println(message);
        });
  }
}
