package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CoordinationLayer;
import frc.robot.subsystems.drive.DriveCoordinator;
import java.util.HashMap;

public class Autos {

  public HashMap<String, AutoAction> autos = new HashMap<>();

  @JSONExclude public HashMap<String, Command> autoCommands = new HashMap<>();

  public void loadAutoCommands(
      DriveCoordinator driveCoordinator, CoordinationLayer coordinationLayer) {
    var context = new AutoAction.AutoActionContext(driveCoordinator, coordinationLayer, this);
    for (var entry : autos.entrySet()) {
      autoCommands.put(entry.getKey(), entry.getValue().toCommand(context));
    }
  }

  public Command getAutoCommand(String name) {
    return autoCommands.get(name);
  }
}
