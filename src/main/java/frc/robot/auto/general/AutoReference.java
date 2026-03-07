package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;

public class AutoReference extends AutoAction {

  public String name;

  @Override
  public Command toCommand(AutoActionContext data) {
    return new Command() {
      public Command command = null;

      @Override
      public void initialize() {
        command = data.autos().getAutoCommand(name);
        if (command == null) {
          throw new RuntimeException("Auto with name '" + name + "' not found.");
        }
        command.initialize();
      }

      @Override
      public void execute() {
        if (command == null) {
          throw new RuntimeException("Command for auto '" + name + "' not initialized.");
        }
        command.execute();
      }

      @Override
      public boolean isFinished() {
        if (command == null) {
          throw new RuntimeException("Command for auto '" + name + "' not initialized.");
        }
        return command.isFinished();
      }

      @Override
      public void end(boolean interrupted) {
        if (command == null) {
          throw new RuntimeException("Command for auto '" + name + "' not initialized.");
        }
        command.end(interrupted);
      }
    };
  }
}
