package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CoordinationLayer;
import frc.robot.auto.coordinationLayer.ClimbHangAction;
import frc.robot.auto.coordinationLayer.ClimbSearchAction;
import frc.robot.auto.coordinationLayer.DeployIntakeAction;
import frc.robot.auto.coordinationLayer.StartShooting;
import frc.robot.auto.coordinationLayer.StopShooting;
import frc.robot.auto.coordinationLayer.StowIntakeAction;
import frc.robot.auto.drive.AutoPilotAction;
import frc.robot.auto.drive.FollowPathPlannerPath;
import frc.robot.auto.drive.StopDriveAction;
import frc.robot.auto.drive.XBasedAutoPilotAction;
import frc.robot.auto.general.AutoReference;
import frc.robot.auto.general.Deadline;
import frc.robot.auto.general.NetworkConfigurableWait;
import frc.robot.auto.general.Parallel;
import frc.robot.auto.general.Print;
import frc.robot.auto.general.Race;
import frc.robot.auto.general.Sequence;
import frc.robot.auto.general.Wait;
import frc.robot.subsystems.drive.DriveCoordinator;
import java.util.HashMap;
import java.util.Map;

@JsonType(
    property = "type",
    subtypes = {
      // General actions
      @JsonSubtype(clazz = AutoReference.class, name = "AutoReference"),
      @JsonSubtype(clazz = Deadline.class, name = "Deadline"),
      @JsonSubtype(clazz = Sequence.class, name = "Sequence"),
      @JsonSubtype(clazz = Parallel.class, name = "Parallel"),
      @JsonSubtype(clazz = Race.class, name = "Race"),
      @JsonSubtype(clazz = Wait.class, name = "Wait"),
      @JsonSubtype(clazz = Print.class, name = "Print"),
      @JsonSubtype(clazz = NetworkConfigurableWait.class, name = "NetworkConfigurableWait"),
      // Drive actions
      @JsonSubtype(clazz = AutoPilotAction.class, name = "AutoPilotAction"),
      @JsonSubtype(clazz = XBasedAutoPilotAction.class, name = "XBasedAutoPilotAction"),
      @JsonSubtype(clazz = StopDriveAction.class, name = "StopDriveAction"),
      @JsonSubtype(clazz = FollowPathPlannerPath.class, name = "FollowPathPlannerPath"),
      // Coordination layer actions
      @JsonSubtype(clazz = DeployIntakeAction.class, name = "DeployIntakeAction"),
      @JsonSubtype(clazz = StowIntakeAction.class, name = "StowIntakeAction"),
      @JsonSubtype(clazz = ClimbSearchAction.class, name = "ClimbSearchAction"),
      @JsonSubtype(clazz = ClimbHangAction.class, name = "ClimbHangAction"),
      @JsonSubtype(clazz = StartShooting.class, name = "StartShooting"),
      @JsonSubtype(clazz = StopShooting.class, name = "StopShooting"),
    })
public abstract class AutoAction {

  /** Maps each concrete AutoAction class to its JSON discriminator, from the @JsonType table. */
  private static final Map<Class<?>, String> TYPE_NAMES = buildTypeNames();

  private static Map<Class<?>, String> buildTypeNames() {
    Map<Class<?>, String> names = new HashMap<>();
    for (JsonSubtype subtype : AutoAction.class.getAnnotation(JsonType.class).subtypes()) {
      names.put(subtype.clazz(), subtype.name());
    }
    return names;
  }

  /**
   * The polymorphic discriminator written to JSON. coppercore's adapter uses @JsonType/@JsonSubtype
   * only to pick the subclass when *deserializing*; on *serialize* it just emits this field. Gson
   * populates it from the JSON when loading (bypassing this initializer via Unsafe), so this
   * default only matters when an action is constructed directly in Java (e.g. the auto generator),
   * where it resolves the discriminator from the runtime class using the @JsonSubtype table above.
   */
  public String type = TYPE_NAMES.get(getClass());

  public record AutoActionContext(
      DriveCoordinator driveCoordinator,
      CoordinationLayer coordinationLayer,
      Autos autos,
      boolean flipped,
      boolean mirrored) {

    public AutoActionContext(
        DriveCoordinator driveCoordinator, CoordinationLayer coordinationLayer, Autos autos) {
      this(driveCoordinator, coordinationLayer, autos, false, false);
    }

    public AutoActionContext flip() {
      return new AutoActionContext(driveCoordinator, coordinationLayer, autos, !flipped, mirrored);
    }

    public AutoActionContext mirror() {
      return new AutoActionContext(driveCoordinator, coordinationLayer, autos, flipped, !mirrored);
    }
  }

  public abstract Command toCommand(AutoActionContext data);

  // ---------------------------------------------------------------------------
  // Fluent combinators for composing actions when authoring autos.
  // ---------------------------------------------------------------------------

  /** Returns a {@link Sequence} that runs this action, then {@code next} in order. */
  public Sequence andThen(AutoAction... next) {
    AutoAction[] all = new AutoAction[next.length + 1];
    all[0] = this;
    System.arraycopy(next, 0, all, 1, next.length);
    return Sequence.of(all);
  }

  /**
   * Returns a {@link Sequence} that runs this action, then runs {@code parallelActions} together in
   * parallel. Sugar for {@code andThen(Parallel.of(parallelActions))} — the parallel grouping is
   * explicit, so there is no ambiguity about what runs alongside what.
   */
  public Sequence andThenInParallel(AutoAction... parallelActions) {
    return andThen(Parallel.of(parallelActions));
  }

  /**
   * Returns a {@link Sequence} that runs this action, then races {@code raceActions} against each
   * other (the group ends when the first finishes). Sugar for {@code
   * andThen(Race.of(raceActions))}.
   */
  public Sequence andThenRacing(AutoAction... raceActions) {
    return andThen(Race.of(raceActions));
  }
}
