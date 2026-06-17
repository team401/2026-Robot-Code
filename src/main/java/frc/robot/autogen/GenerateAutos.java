package frc.robot.autogen;

import static frc.robot.autogen.Dsl.apConstraints;
import static frc.robot.autogen.Dsl.apProfile;
import static frc.robot.autogen.Dsl.autoPilotTo;
import static frc.robot.autogen.Dsl.climbHang;
import static frc.robot.autogen.Dsl.climbSearch;
import static frc.robot.autogen.Dsl.degrees;
import static frc.robot.autogen.Dsl.deployIntake;
import static frc.robot.autogen.Dsl.followPath;
import static frc.robot.autogen.Dsl.inParallel;
import static frc.robot.autogen.Dsl.meters;
import static frc.robot.autogen.Dsl.networkConfigurableWait;
import static frc.robot.autogen.Dsl.pidGains;
import static frc.robot.autogen.Dsl.pose2d;
import static frc.robot.autogen.Dsl.rotation2d;
import static frc.robot.autogen.Dsl.seconds;
import static frc.robot.autogen.Dsl.seq;
import static frc.robot.autogen.Dsl.startShooting;
import static frc.robot.autogen.Dsl.stopShooting;
import static frc.robot.autogen.Dsl.stowIntake;
import static frc.robot.autogen.Dsl.waitSeconds;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APTarget;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfig;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.json.helpers.JSONConverter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.auto.Auto;
import frc.robot.auto.AutoAction;
import frc.robot.auto.Autos;
import frc.robot.auto.general.Sequence;
import frc.robot.util.json.JSONAPTarget;

/**
 * Builds every auto routine in Java and serializes them to the Autos.json format using the robot's
 * own coppercore Gson configuration. Replaces the former Python auto generator.
 *
 * <p>Sequencing uses {@code seq.andThen(...)} (or {@code action.andThen(...)}); parallel/race
 * groups are built explicitly with {@code inParallel(...)} / {@code inRace(...)} so grouping is
 * never ambiguous, e.g. {@code seq.andThen(a, inParallel(b, c))}. Conditional sequences accumulate
 * with {@code result = result.andThen(...)} (andThen flattens, so this just appends).
 *
 * <p>Usage: {@code java frc.robot.autogen.GenerateAutos [environment] [outputFile]}.
 */
public final class GenerateAutos {

  // TODO: Add alliance-relative coordinate utilities.
  // TODO: Replace placeholder coordinates with real field positions.
  // TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.

  private static final double DEFAULT_TRENCH_VELOCITY = 4.2;
  private static final double AGGRESSIVE_TRENCH_VELOCITY = 5.1;
  private static final Pose2d LEFT_TRENCH_CENTER_SIDE_POSE = pose2d(5.2, 7.4, -90);

  // Shooting position in the middle of the left side of the alliance zone.
  // This is where we end up after coming over the bump.
  private static final Pose2d LEFT_ALLIANCE_ZONE_MIDDLE_POSE = pose2d(2.700, 5.75, -90);

  private static final APConstraints CLIMB_CONSTRAINTS = apConstraints(2.0, 2.0, 0.1);

  public static void main(String[] args) throws java.io.IOException {
    String environment = args.length > 0 ? args[0] : "comp";
    // The JSON is written to this file rather than stdout: initializing HAL prints native
    // diagnostics to stdout, which would otherwise corrupt the serialized output.
    String outputFile = args.length > 1 ? args[1] : null;

    Field.initialize(environment);

    Autos autos = build();

    JSONConverter.addConversion(APTarget.class, JSONAPTarget.class);
    JSONSyncConfig config = new JSONSyncConfigBuilder().build();
    JSONSync<Autos> sync = new JSONSync<>(autos, "", config);
    String json = sync.serialize();

    if (outputFile != null) {
      java.nio.file.Files.writeString(java.nio.file.Path.of(outputFile), json);
    } else {
      System.out.println(json);
    }
  }

  private static Autos build() {
    Autos autos = new Autos();

    autos.autos.put(
        "Literally just shoot Auto", new Auto(seq.andThen(startShooting()), false, false));

    autos.autos.put("Aggressive Depot", auto(seq.andThen(aggressive(true, false, false, true))));
    autos.autos.put(
        "Aggressive No Depot", auto(seq.andThen(aggressive(false, false, false, true))));
    autos.autos.put(
        "Aggressive Depot From Bump", auto(seq.andThen(aggressive(true, true, true, false))));

    autos.autos.put(
        "Conservative Depot", auto(seq.andThen(conservative(true, false, false, true))));
    autos.autos.put(
        "Conservative No Depot", auto(seq.andThen(conservative(false, false, false, true))));
    autos.autos.put(
        "Conservative Depot From Bump", auto(seq.andThen(conservative(true, true, true, false))));

    autos.autos.put("Follower", auto(follower()));

    autos.autos.put(
        "Single Swipe Then Depot",
        new Auto(
            seq.andThen(
                aggressive(false, false, false, false),
                goToDepotAndIntake(),
                waitSeconds(1.0),
                cycleIntake(6.0 / 3.0, 6)),
            false,
            true));

    autos.autos.put(
        "Center Depot",
        new Auto(
            seq.andThen(
                deployIntake(),
                startShooting(),
                networkConfigurableWait("Center Depot - Shoot Preload", seconds(4.0)),
                goToDepotAndIntake(),
                // startShooting();
                waitSeconds(2.0),
                cycleIntake(6.0 / 3.0, 6)),
            false,
            true));

    autos.routines.put("LeftClimb", climb(Field.leftClimbLocation(), rotation2d(0)));
    autos.routines.put("RightClimb", climb(Field.rightClimbLocation(), rotation2d(0)));

    return autos;
  }

  /** A mirrorable, flippable auto (the @auto defaults). */
  private static Auto auto(AutoAction root) {
    return new Auto(root, true, true);
  }

  // ---------------------------------------------------------------------------
  // Reusable command builders (ported from autos.py / routines.py)
  // ---------------------------------------------------------------------------

  private static AutoAction cycleIntake(double time, int count) {
    double delayEach = time / 2 / count;
    Sequence result = seq;
    for (int i = 0; i < count; i++) {
      result =
          result.andThen(
              stowIntake(), waitSeconds(delayEach), deployIntake(), waitSeconds(delayEach));
    }
    return result;
  }

  private static AutoAction fromBumpPrepareForTrench(double angle) {
    return seq.andThen(
        autoPilotTo(3.5, 7.55, angle)
            .withVelocity(DEFAULT_TRENCH_VELOCITY)
            .withEntryAngle(0)
            .withConstraints(apConstraints(2.0, 2.0, 2.0))
            .withPidGains(pidGains(1.5))
            .toAutoAction());
  }

  private static AutoAction goToCenterUnderLeftTrenchFromAllianceIntakeIn() {
    return seq.andThen(
        autoPilotTo(4.0, 7.55, -90)
            .withVelocity(DEFAULT_TRENCH_VELOCITY)
            .withEntryAngle(0)
            .toXBasedAutoAction(),
        // xBasedAutopilot(LEFT_TRENCH_CENTER_SIDE_POSE transformed by rotation,
        //   velocity, entryAngle=secondEntryAngle)
        followPath("Left Trench To Center Intake In"));
  }

  private static AutoAction goToDepotAndIntake() {
    // drive near the depot and slow down so that we don't go too fast while intaking
    return seq.andThen(
        autoPilotTo(1.5, 5.1, 135)
            .withVelocity(1.0)
            .withProfile(
                apProfile(apConstraints(5.1, 10.0, 3.0), meters(0.1), degrees(4.0), meters(0.2)))
            .toAutoAction(),
        autoPilotTo(0.715, 5.1, 135)
            .withConstraints(apConstraints(1.0, 3.0, 3.0))
            .withPidGains(pidGains(1.5))
            .toAutoAction(),
        // Wait to stop autopilot from seeing initial velocity and panicking.
        // Autopilot is probably more robust and simpler. Also removes a path planner
        // path and enables us to use hot reload to tune it: followPath("Intake Depot").
        waitSeconds(0.1),
        autoPilotTo(0.715, 6.4, 135)
            .withConstraints(apConstraints(1.0, 3.0, 2.0))
            .withPidGains(pidGains(1.5))
            .toAutoAction(),
        // Wait to stop autopilot from seeing initial velocity and panicking.
        waitSeconds(0.1),
        autoPilotTo(1.0, 6.8, 135)
            .withConstraints(apConstraints(5.1, 5.0, 3.0))
            .withPidGains(pidGains(1.5))
            .toAutoAction());
  }

  private static AutoAction aggressive(
      boolean useDepot, boolean fromBump, boolean shootPreload, boolean doSecondSweep) {
    double intakeCycleTime = 1.0 / 3.0;
    int intakeCycleCount = 1;

    Sequence result = seq;
    if (shootPreload) {
      result = result.andThen(startShooting(), waitSeconds(1.5));
    }
    if (fromBump) {
      result = result.andThen(fromBumpPrepareForTrench(-90));
    }
    if (shootPreload) {
      result = result.andThen(stopShooting());
    }

    // Cycle 1
    result =
        result.andThen(
            // Autopilot under the trench.
            // Gives us solid acceleration and makes us resilient to unpredictable starting
            // location.
            autoPilotTo(LEFT_TRENCH_CENTER_SIDE_POSE)
                .withVelocity(AGGRESSIVE_TRENCH_VELOCITY)
                // Constraints here are unique to this very high speed, high
                // acceleration movement, so almost infinite acceleration limit is
                // hardcoded in here.
                .withConstraints(apConstraints(AGGRESSIVE_TRENCH_VELOCITY, 200.0))
                // No entry angle, we just want to beeline to trench exit.
                .toXBasedAutoAction(),
            // inParallel(stowIntake(),
            //     followPath("Starting Position Left Trench To Center Intake In"))
            inParallel(deployIntake(), followPath("Left Side Aggressive Sweep Intake In")),
            inParallel(
                seq.andThen(waitSeconds(0.6), startShooting()),
                followPath("Left Bump To Alliance")),
            waitSeconds(0.1),
            autoPilotTo(LEFT_ALLIANCE_ZONE_MIDDLE_POSE).toAutoAction(),
            cycleIntake(2.5, 5));

    if (doSecondSweep) {
      result =
          result.andThen(
              cycleIntake(intakeCycleTime, intakeCycleCount),
              // wait(1.0),
              // Cycle 2
              autoPilotTo(3.5, 7.55, -90)
                  .withVelocity(DEFAULT_TRENCH_VELOCITY)
                  .withEntryAngle(0)
                  .withConstraints(apConstraints(2.0, 2.0, 2.0))
                  .withPidGains(pidGains(1.5))
                  .toAutoAction(),
              inParallel(stopShooting(), goToCenterUnderLeftTrenchFromAllianceIntakeIn()),
              inParallel(deployIntake(), followPath("Left Side Close 2nd Sweep Intake In")),
              inParallel(
                  seq.andThen(waitSeconds(0.4), startShooting()),
                  followPath("Left Bump To Alliance")),
              waitSeconds(0.1),
              autoPilotTo(2.700, 5.75, -90).toAutoAction(),
              waitSeconds(1));
    }

    if (useDepot) {
      result =
          result.andThen(
              autoPilotTo(1.5, 5.9, -180)
                  .withVelocity(0.0)
                  .withConstraints(apConstraints(2.0, 2.0, 2.0))
                  .withPidGains(pidGains(1.5))
                  .toAutoAction());
    }

    return result.andThen(cycleIntake(intakeCycleTime, intakeCycleCount));
  }

  private static AutoAction conservative(
      boolean useDepot, boolean fromBump, boolean shootPreload, boolean doSecondSweep) {
    double intakeCycleTime = 0.5;
    int intakeCycleCount = 1;

    Sequence result = seq;
    if (shootPreload) {
      result = result.andThen(startShooting(), waitSeconds(1.5));
    }
    if (fromBump) {
      result = result.andThen(fromBumpPrepareForTrench(0));
    }
    if (shootPreload) {
      result = result.andThen(stopShooting());
    }

    // Cycle 1
    result =
        result.andThen(
            autoPilotTo(5.2, 7.4).withVelocity(DEFAULT_TRENCH_VELOCITY).toXBasedAutoAction(),
            inParallel(stowIntake(), followPath("Starting Position Left Trench To Center")),
            inParallel(deployIntake(), followPath("Left Side Conservative Sweep")),
            inParallel(
                seq.andThen(waitSeconds(0.6), startShooting()),
                followPath("Left Bump To Alliance")),
            waitSeconds(0.1),
            autoPilotTo(2.700, 5.75, -90).toAutoAction(),
            waitSeconds(2.5));

    // wait(1.0),
    // Cycle 2
    if (doSecondSweep) {
      result =
          result.andThen(
              cycleIntake(intakeCycleTime, intakeCycleCount),
              autoPilotTo(3.5, 7.55, -90)
                  .withVelocity(DEFAULT_TRENCH_VELOCITY)
                  .withEntryAngle(0)
                  .withConstraints(apConstraints(2.0, 2.0, 2.0))
                  .withPidGains(pidGains(1.5))
                  .toAutoAction(),
              inParallel(stopShooting(), goToCenterUnderLeftTrenchFromAllianceIntakeIn()),
              inParallel(deployIntake(), followPath("Left Side Close 2nd Sweep Intake In")),
              inParallel(
                  seq.andThen(waitSeconds(0.4), startShooting()),
                  followPath("Left Bump To Alliance")),
              waitSeconds(0.1),
              autoPilotTo(2.700, 5.75, -90).toAutoAction(),
              waitSeconds(1));
    }

    if (useDepot) {
      result =
          result.andThen(
              autoPilotTo(1.5, 5.9, -180)
                  .withVelocity(0.0)
                  .withConstraints(apConstraints(2.0, 2.0, 2.0))
                  .withPidGains(pidGains(1.5))
                  .toAutoAction());
    }

    return result.andThen(cycleIntake(intakeCycleTime, intakeCycleCount));
  }

  private static AutoAction follower() {
    // Don't deploy the intake to avoid smashing it into the trench.
    // deployIntake();
    // startShooting();
    return seq.andThen(
        networkConfigurableWait("Follower - Preload", seconds(2.5)),
        // stopShooting();
        autoPilotTo(LEFT_TRENCH_CENTER_SIDE_POSE)
            .withVelocity(2.0)
            // Constraints here are unique to this very high speed, high
            // acceleration movement, so almost infinite acceleration limit is
            // hardcoded in here.
            .withConstraints(apConstraints(AGGRESSIVE_TRENCH_VELOCITY, 200.0))
            // No entry angle, we just want to beeline to trench exit.
            .toXBasedAutoAction(),
        deployIntake(),
        followPath("Left Side Follower Sweep Intake In"),
        networkConfigurableWait("Follower - Before Bump Return", seconds(0.0)),
        waitSeconds(0.1),
        autoPilotTo(2.700, 5.75, -90).toAutoAction(),
        autoPilotTo(1.5, 5.1, 135)
            .withProfile(
                apProfile(apConstraints(5.1, 10.0, 3.0), meters(0.1), degrees(4.0), meters(0.2)))
            .toAutoAction(),
        startShooting(),
        networkConfigurableWait("Follower - Before Depot", seconds(0.0)),
        goToDepotAndIntake(),
        waitSeconds(2.0),
        cycleIntake(6.0 / 3.0, 6));
  }

  /** Reusable climb lineup subroutine. */
  private static AutoAction climb(Pose2d targetPose, Rotation2d entryAngle) {
    // xBasedAutopilot(FieldConstants.Alliance.center transformed by
    //   climb_offset, constraints=CLIMB_CONSTRAINTS)
    return seq.andThen(
        inParallel(
            seq.andThen(
                autoPilotTo(targetPose)
                    .withEntryAngle(entryAngle)
                    .withConstraints(CLIMB_CONSTRAINTS)
                    .toXBasedAutoAction()),
            climbSearch()),
        waitSeconds(0.5),
        climbHang());
  }

  private GenerateAutos() {}
}
