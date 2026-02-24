package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;

import java.util.Objects;
import java.util.stream.Stream;

public class Deadline extends AutoAction {

  public AutoAction deadline;

  public AutoAction[] others;

  @Override
  public Command toCommand(AutoActionData data) {
    Objects.requireNonNull(deadline, "deadline command cant be null");
    Objects.requireNonNull(others, "other commands cant be null");
    return deadline
        .toCommand(data)
        .deadlineFor(
            Stream.of(others).map(action -> action.toCommand(data)).toArray(Command[]::new));
  }
}
