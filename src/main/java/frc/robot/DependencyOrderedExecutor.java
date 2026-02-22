package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * The DependencyOrderedExecutor handles the order of all actions that take place in periodic based
 * on the dependencies that they have declared. It runs each action in an order such that all of its
 * dependencies have been executed beforehand.
 *
 * <p>Subsystems must register methods to update inputs or apply outputs by calling
 * `registerAction`. Then, they can call `addDependencies` by passing that action and its
 * dependencies. Actions are represented by an ActionKey object that stores their unique ID. This
 * means that each subsystem can keep track of its own actions and provide a handle to those actions
 * by the use of static methods to other classes. After all actions and dependencies have been
 * registered, call `finalizeSchedule`, which will generate the dependency graph and execution
 * order. After this, calling `execute` will execute all actions in order.
 */
public class DependencyOrderedExecutor {
  /** The ActionKey class represents a unique identifier for a DependencyOrderedExecutor action. */
  public record ActionKey(String name) {
    @Override
    public String toString() {
      return "ActionKey(" + name + ")";
    }
  }

  private record NamedAction(String name, Runnable action) {}

  private final HashMap<ActionKey, Runnable> actionMap = new HashMap<>();
  private final DirectedAcyclicGraph<ActionKey> dependencyGraph = new DirectedAcyclicGraph<>();

  /**
   * An Optional containing either the actions in the order in which they should be run, if the
   * schedule has been finalized, or empty if the schedule has not yet been finalized.
   */
  private Optional<List<NamedAction>> actionSchedule = Optional.empty();

  private final Watchdog watchdog;

  /**
   * Create a new DependencyOrderedExecutor with the default period of 0.02 seconds
   *
   * <p>See {@link DependencyOrderedExecutor#DependencyOrderedExecutor(Time)} to customize the
   * period.
   */
  public DependencyOrderedExecutor() {
    watchdog = new Watchdog(Seconds.of(TimedRobot.kDefaultPeriod), () -> {});
  }

  /**
   * Create a new DependencyOrderedExecutor with a custom loop period
   *
   * @param period A Time, defining how long each loop should take
   */
  public DependencyOrderedExecutor(Time period) {
    watchdog = new Watchdog(period, () -> {});
  }

  /**
   * Register an action to run in each periodic, setting the Runnable associated with an ActionKey
   *
   * @param key The key of the action to register
   * @param action The Runnable to run in order to execute this action
   */
  public void registerAction(ActionKey key, Runnable action) {
    checkForModificationAfterFinalization();
    if (actionMap.containsKey(key)) {
      throw new IllegalArgumentException(
          "Key " + key.name() + " was used in registerAction after already being registered.");
    }

    actionMap.put(key, action);
    dependencyGraph.addNode(key);
  }

  public void addDependencies(ActionKey action, ActionKey... dependencies) {
    checkForModificationAfterFinalization();

    for (var dependency : dependencies) {
      dependencyGraph.addEdge(dependency, action);
    }
  }

  /**
   * Finalize the schedule, solving all dependencies and generating an action schedule that runs
   * actions in a correct order.
   */
  public void finalizeSchedule() {
    System.out.println("[DOE] Finalizing schedule: ");
    System.out.println("[DOE] Dependency graph:");
    dependencyGraph.printMermaid(new PrintWriter(System.out, true));
    dependencyGraph.printDOT(new PrintWriter(System.out, true));

    System.out.println("[DOE] Finalized schedule:");
    actionSchedule =
        Optional.of(
            dependencyGraph.topologicalSort().stream()
                .map(
                    key -> {
                      System.out.println(" - " + key.name());
                      if (!actionMap.containsKey(key)) {
                        throw new IllegalStateException("No action was registered for " + key);
                      }
                      return new NamedAction(key.name(), actionMap.get(key));
                    })
                .collect(Collectors.toList()));
  }

  /**
   * Check whether the schedule has been finalized and, if so, throw an error.
   *
   * <p>This method should be called at the start of all methods which modify the dependency graph.
   */
  private void checkForModificationAfterFinalization() {
    if (isFinalized()) {
      throw new UnsupportedOperationException(
          "Executor actions or dependencies were modified after schedule was finalized.");
    }
  }

  /**
   * Check whether this executor has been finalized. If this method returns true, no new actions or
   * dependencies can be added.
   *
   * @return True if the schedule is finalized, false if not.
   */
  public boolean isFinalized() {
    return actionSchedule.isPresent();
  }

  /**
   * Execute all scheduled actions, in order.
   *
   * <p>If the action schedule hasn't been finalized, this method will print a warning and return.
   * It does not crash code in case a cycle is required before initialization is complete.
   */
  public void execute() {
    watchdog.reset();
    actionSchedule.ifPresentOrElse(
        this::runActions,
        () ->
            new Exception(
                    "Warning: DependencyOrderedInjector.execute() called before schedule was finalized.")
                .printStackTrace());

    watchdog.disable();
    if (watchdog.isExpired()) {
      System.out.println("DependencyOrderedExecutor loop overrun");
      watchdog.printEpochs();
    }
  }

  /**
   * Run a sequence of actions and add their epochs to the watchdog, which will keep track of the
   * time taken by each epoch (each action) and then report that time in the case of a loop overrun.
   *
   * @param actions The List of NamedActions to run
   */
  private void runActions(List<NamedAction> actions) {
    for (var namedAction : actions) {
      namedAction.action().run();
      watchdog.addEpoch(namedAction.name());
    }
  }
}
