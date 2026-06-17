package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoAction;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;

public class Sequence extends AutoAction {

  public AutoAction[] actions;

  /** Creates a Sequence of the given actions. */
  public static Sequence of(AutoAction... actions) {
    Sequence sequence = new Sequence();
    sequence.actions = actions;
    return sequence;
  }

  /** Creates a Sequence from a list of actions (convenient for conditional building). */
  public static Sequence of(List<AutoAction> actions) {
    return of(actions.toArray(new AutoAction[0]));
  }

  /** Appends {@code next} to this sequence, returning a new flattened Sequence. */
  @Override
  public Sequence andThen(AutoAction... next) {
    AutoAction[] all = new AutoAction[actions.length + next.length];
    System.arraycopy(actions, 0, all, 0, actions.length);
    System.arraycopy(next, 0, all, actions.length, next.length);
    return of(all);
  }

  @Override
  public Command toCommand(AutoActionContext data) {
    Objects.requireNonNull(actions, "actions cannot be null");
    return new SequentialCommandGroup(
        Stream.of(actions).map(action -> action.toCommand(data)).toArray(Command[]::new));
  }
}
