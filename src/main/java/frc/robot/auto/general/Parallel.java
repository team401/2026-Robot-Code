package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoAction;
import java.util.stream.Stream;

public class Parallel extends AutoAction {
  public AutoAction[] actions;

  @Override
  public void setupAction(AutoActionData data) {
    for (AutoAction action : actions) {
      action.setupAction(data);
    }
  }

  @Override
  public Command toCommand(AutoActionData data) {
    return new ParallelCommandGroup(
        Stream.of(actions).map(action -> action.toCommand(data)).toArray(Command[]::new));
  }
}
