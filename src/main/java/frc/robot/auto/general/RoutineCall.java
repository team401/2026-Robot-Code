package frc.robot.auto.general;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;
import frc.robot.auto.RoutineRegistry;

public class RoutineCall extends AutoAction {

  String routineName;
  @JSONExclude AutoAction routine;

  @Override
  public void setupAction(AutoActionData data) {
    routine = RoutineRegistry.getRoutine(routineName);
    if (routine == null) {
      throw new IllegalStateException("Routine with name " + routineName + " not found");
    }
    routine.setupAction(data);
  }

  @Override
  public Command toCommand(AutoActionData data) {
    return routine.toCommand(data);
  }
}
