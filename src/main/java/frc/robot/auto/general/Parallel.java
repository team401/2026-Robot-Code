package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoAction;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;

public class Parallel extends AutoAction {
  public AutoAction[] actions;

  /** Creates a Parallel of the given actions. */
  public static Parallel of(AutoAction... actions) {
    Parallel parallel = new Parallel();
    parallel.actions = actions;
    return parallel;
  }

  /** Creates a Parallel from a list of actions (convenient for conditional building). */
  public static Parallel of(List<AutoAction> actions) {
    return of(actions.toArray(new AutoAction[0]));
  }

  /** Adds {@code others} to run alongside this parallel, returning a new flattened Parallel. */
  @Override
  public Parallel inParallelWith(AutoAction... others) {
    AutoAction[] all = new AutoAction[actions.length + others.length];
    System.arraycopy(actions, 0, all, 0, actions.length);
    System.arraycopy(others, 0, all, actions.length, others.length);
    return of(all);
  }

  @Override
  public Command toCommand(AutoActionContext data) {
    Objects.requireNonNull(actions, "actions cannot be null");
    return new ParallelCommandGroup(
        Stream.of(actions).map(action -> action.toCommand(data)).toArray(Command[]::new));
  }
}
