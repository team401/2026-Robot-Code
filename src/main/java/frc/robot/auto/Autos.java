package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CoordinationLayer;
import frc.robot.subsystems.drive.DriveCoordinator;
import java.util.HashMap;

public class Autos {

  public HashMap<String, AutoAction> autos = new HashMap<>();
  public HashMap<String, AutoAction> routines = new HashMap<>();

  @JSONExclude public HashMap<String, Command> autoCommands = new HashMap<>();
  @JSONExclude public HashMap<String, Command> routineCommands = new HashMap<>();

  public void loadAutoCommands(
      DriveCoordinator driveCoordinator, CoordinationLayer coordinationLayer) {
    var context = new AutoAction.AutoActionContext(driveCoordinator, coordinationLayer, this);
    for (var entry : autos.entrySet()) {
      autoCommands.put(entry.getKey(), entry.getValue().toCommand(context));
    }
    for (var entry : routines.entrySet()) {
      routineCommands.put(entry.getKey(), entry.getValue().toCommand(context));
    }
  }

  public Command getAutoCommand(String name) {
    return autoCommands.get(name);
  }

  public Command getRoutineCommand(String name) {
    return routineCommands.get(name);
  }

  public Command getRoutineCommandReference(String name) {
    return new Command() {
      public Command command = null;

      @Override
      public void initialize() {
        command = getRoutineCommand(name);
        if (command == null) {
          throw new RuntimeException("Routine with name '" + name + "' not found.");
        }
        command.initialize();
      }

      @Override
      public void execute() {
        if (command == null) {
          throw new RuntimeException("Command for routine '" + name + "' not initialized.");
        }
        command.execute();
      }

      @Override
      public boolean isFinished() {
        if (command == null) {
          throw new RuntimeException("Command for routine '" + name + "' not initialized.");
        }
        return command.isFinished();
      }

      @Override
      public void end(boolean interrupted) {
        if (command == null) {
          throw new RuntimeException("Command for routine '" + name + "' not initialized.");
        }
        command.end(interrupted);
      }
    };
  }
}
