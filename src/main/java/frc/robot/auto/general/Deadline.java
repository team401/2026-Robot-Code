package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;
import java.util.stream.Stream;

public class Deadline extends AutoAction {

  public AutoAction deadline;

  public AutoAction[] others;

  @Override
  public void setupParameters(AutoActionData data) {
    if (deadline != null) {
      deadline.setupParameters(data);
    }
    if (others != null) {
      for (AutoAction action : others) {
        action.setupParameters(data);
      }
    }
  }

  @Override
  public Command toCommand(AutoActionData data) {
    return deadline
        .toCommand(data)
        .deadlineFor(
            Stream.of(others).map(action -> action.toCommand(data)).toArray(Command[]::new));
  }
}
