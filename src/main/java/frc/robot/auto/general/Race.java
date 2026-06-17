package frc.robot.auto.general;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.auto.AutoAction;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;

public class Race extends AutoAction {
  private AutoAction[] actions;

  /** Creates a Race of the given actions (all run until the first finishes). */
  public static Race of(AutoAction... actions) {
    Race race = new Race();
    race.actions = actions;
    return race;
  }

  /** Creates a Race from a list of actions. */
  public static Race of(List<AutoAction> actions) {
    return of(actions.toArray(new AutoAction[0]));
  }

  @Override
  public Command toCommand(AutoActionContext data) {
    Objects.requireNonNull(actions, "actions cannot be null");
    return new ParallelRaceGroup(
        Stream.of(actions).map(action -> action.toCommand(data)).toArray(Command[]::new));
  }
}
