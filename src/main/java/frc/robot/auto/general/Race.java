package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.auto.AutoAction;
import java.util.stream.Stream;

public class Race extends AutoAction {
  public AutoAction[] actions;

  @Override
  public void setupParameters(AutoActionData data) {
    for (AutoAction action : actions) {
      action.setupParameters(data);
    }
  }

  @Override
  public Command toCommand(AutoActionData data) {
    return new ParallelRaceGroup(
        Stream.of(actions).map(action -> action.toCommand(data)).toArray(Command[]::new));
  }
}
