package frc.robot.autogen;

import static frc.robot.autogen.Dsl.ap;
import static frc.robot.autogen.Dsl.apConstraints;
import static frc.robot.autogen.Dsl.apProfile;
import static frc.robot.autogen.Dsl.climbHang;
import static frc.robot.autogen.Dsl.climbSearch;
import static frc.robot.autogen.Dsl.degrees;
import static frc.robot.autogen.Dsl.deployIntake;
import static frc.robot.autogen.Dsl.followPath;
import static frc.robot.autogen.Dsl.meters;
import static frc.robot.autogen.Dsl.networkConfigurableWait;
import static frc.robot.autogen.Dsl.parallel;
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
import frc.robot.util.json.JSONAPTarget;
import java.util.ArrayList;
import java.util.List;

/**
 * Builds every auto routine in Java and serializes them to the Autos.json format using the robot's
 * own coppercore Gson configuration. Replaces the former Python auto generator.
 *
 * <p>Usage: {@code java frc.robot.autogen.AutosGen [environment]} — prints the JSON to stdout.
 */
public final class AutosGen {

  // TODO: Add alliance-relative coordinate utilities.
  // TODO: Replace placeholder coordinates with real field positions.
  // TODO: Switch to AutoPilotAction with entry angle and exit velocity for trench segments.

  // --- constants ported from the old constants.py ---------------------------
  // TODO: Maybe make these loaded from the constants files in the main robot code
  // instead of hardcoded here, to avoid duplication and potential inconsistencies.
  // (The climb poses below are now derived from the robot's FieldConstants; see Field.)
  private static final double DEFAULT_TRENCH_VELOCITY = 4.2;
  private static final double AGGRESSIVE_TRENCH_VELOCITY = 5.1;
  private static final Pose2d LEFT_TRENCH_CENTER_SIDE_POSE = pose2d(5.2, 7.4, -90);

  // Shooting position in the middle of the left side of the alliance zone.
  // This is where we end up after coming over the bump.
  private static final Pose2d LEFT_ALLIANCE_ZONE_MIDDLE_POSE = pose2d(2.700, 5.75, -90);

  private static final APConstraints CLIMB_CONSTRAINTS = apConstraints(2.0, 2.0, 0.1);

  public static void main(String[] args) {
    String environment = args.length > 0 ? args[0] : "comp";
    Field.initialize(environment);

    Autos autos = build();

    JSONConverter.addConversion(APTarget.class, JSONAPTarget.class);
    JSONSyncConfig config = new JSONSyncConfigBuilder().build();
    JSONSync<Autos> sync = new JSONSync<>(autos, "", config);
    System.out.println(sync.serialize());
  }

  private static Autos build() {
    Autos autos = new Autos();

    autos.autos.put("Literally just shoot Auto", new Auto(seq(startShooting()), false, false));

    autos.autos.put("Aggressive Depot", auto(seq(aggressive(true, false, false, true))));
    autos.autos.put("Aggressive No Depot", auto(seq(aggressive(false, false, false, true))));
    autos.autos.put("Aggressive Depot From Bump", auto(seq(aggressive(true, true, true, false))));

    autos.autos.put("Conservative Depot", auto(seq(conservative(true, false, false, true))));
    autos.autos.put("Conservative No Depot", auto(seq(conservative(false, false, false, true))));
    autos.autos.put(
        "Conservative Depot From Bump", auto(seq(conservative(true, true, true, false))));

    autos.autos.put("Follower", auto(follower()));

    autos.autos.put(
        "Single Swipe Then Depot",
        new Auto(
            seq(
                aggressive(false, false, false, false),
                goToDepotAndIntake(),
                waitSeconds(1.0),
                cycleIntake(6.0 / 3.0, 6)),
            false,
            true));

    autos.autos.put(
        "Center Depot",
        new Auto(
            seq(
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
    List<AutoAction> actions = new ArrayList<>();
    for (int i = 0; i < count; i++) {
      actions.add(stowIntake());
      actions.add(waitSeconds(delayEach));
      actions.add(deployIntake());
      actions.add(waitSeconds(delayEach));
    }
    return seq(actions);
  }

  private static AutoAction fromBumpPrepareForTrench(double angle) {
    return seq(
        ap().pose(3.5, 7.55, angle)
            .velocity(DEFAULT_TRENCH_VELOCITY)
            .entryAngle(0)
            .constraints(apConstraints(2.0, 2.0, 2.0))
            .pidGains(pidGains(1.5))
            .ap());
  }

  private static AutoAction goToCenterUnderLeftTrenchFromAllianceIntakeIn() {
    return seq(
        ap().pose(4.0, 7.55, -90).velocity(DEFAULT_TRENCH_VELOCITY).entryAngle(0).xap(),
        // xBasedAutopilot(LEFT_TRENCH_CENTER_SIDE_POSE transformed by rotation,
        //   velocity, entryAngle=secondEntryAngle)
        followPath("Left Trench To Center Intake In"));
  }

  private static AutoAction goToDepotAndIntake() {
    // drive near the depot and slow down so that we don't go too fast while intaking
    return seq(
        ap().pose(1.5, 5.1, 135)
            .velocity(1.0)
            .profile(
                apProfile(apConstraints(5.1, 10.0, 3.0), meters(0.1), degrees(4.0), meters(0.2)))
            .ap(),
        ap().pose(0.715, 5.1, 135)
            .constraints(apConstraints(1.0, 3.0, 3.0))
            .pidGains(pidGains(1.5))
            .ap(),
        // Wait to stop autopilot from seeing initial velocity and panicking.
        // Autopilot is probably more robust and simpler. Also removes a path planner
        // path and enables us to use hot reload to tune it: followPath("Intake Depot").
        waitSeconds(0.1),
        ap().pose(0.715, 6.4, 135)
            .constraints(apConstraints(1.0, 3.0, 2.0))
            .pidGains(pidGains(1.5))
            .ap(),
        // Wait to stop autopilot from seeing initial velocity and panicking.
        waitSeconds(0.1),
        ap().pose(1.0, 6.8, 135)
            .constraints(apConstraints(5.1, 5.0, 3.0))
            .pidGains(pidGains(1.5))
            .ap());
  }

  private static AutoAction aggressive(
      boolean useDepot, boolean fromBump, boolean shootPreload, boolean doSecondSweep) {
    double intakeCycleTime = 1.0 / 3.0;
    int intakeCycleCount = 1;
    List<AutoAction> a = new ArrayList<>();

    if (shootPreload) {
      a.add(startShooting());
      a.add(waitSeconds(1.5));
    }
    if (fromBump) {
      a.add(fromBumpPrepareForTrench(-90));
    }
    if (shootPreload) {
      a.add(stopShooting());
    }

    // Cycle 1

    // Autopilot under the trench.
    // Gives us solid acceleration and makes us resilient to unpredictable starting
    // location.
    a.add(
        ap().pose(LEFT_TRENCH_CENTER_SIDE_POSE)
            .velocity(AGGRESSIVE_TRENCH_VELOCITY)
            // Constraints here are unique to this very high speed, high
            // acceleration movement, so almost infinite acceleration limit is
            // hardcoded in here.
            .constraints(apConstraints(AGGRESSIVE_TRENCH_VELOCITY, 200.0))
            // No entry angle, we just want to beeline to trench exit.
            .xap());

    // with parallel():
    //   stowIntake();
    //   followPath("Starting Position Left Trench To Center Intake In");

    a.add(parallel(deployIntake(), followPath("Left Side Aggressive Sweep Intake In")));
    a.add(parallel(seq(waitSeconds(0.6), startShooting()), followPath("Left Bump To Alliance")));
    a.add(waitSeconds(0.1));
    a.add(ap().pose(LEFT_ALLIANCE_ZONE_MIDDLE_POSE).ap());
    a.add(cycleIntake(2.5, 5));

    if (doSecondSweep) {
      a.add(cycleIntake(intakeCycleTime, intakeCycleCount));

      // wait(1.0),
      // Cycle 2
      a.add(
          ap().pose(3.5, 7.55, -90)
              .velocity(DEFAULT_TRENCH_VELOCITY)
              .entryAngle(0)
              .constraints(apConstraints(2.0, 2.0, 2.0))
              .pidGains(pidGains(1.5))
              .ap());
      a.add(parallel(stopShooting(), goToCenterUnderLeftTrenchFromAllianceIntakeIn()));
      a.add(parallel(deployIntake(), followPath("Left Side Close 2nd Sweep Intake In")));
      a.add(parallel(seq(waitSeconds(0.4), startShooting()), followPath("Left Bump To Alliance")));
      a.add(waitSeconds(0.1));
      a.add(ap().pose(2.700, 5.75, -90).ap());
      a.add(waitSeconds(1));
    }

    if (useDepot) {
      a.add(
          ap().pose(1.5, 5.9, -180)
              .velocity(0.0)
              .constraints(apConstraints(2.0, 2.0, 2.0))
              .pidGains(pidGains(1.5))
              .ap());
    }

    a.add(cycleIntake(intakeCycleTime, intakeCycleCount));
    return seq(a);
  }

  private static AutoAction conservative(
      boolean useDepot, boolean fromBump, boolean shootPreload, boolean doSecondSweep) {
    double intakeCycleTime = 0.5;
    int intakeCycleCount = 1;
    List<AutoAction> a = new ArrayList<>();

    if (shootPreload) {
      a.add(startShooting());
      a.add(waitSeconds(1.5));
    }
    if (fromBump) {
      a.add(fromBumpPrepareForTrench(0));
    }
    if (shootPreload) {
      a.add(stopShooting());
    }

    // Cycle 1

    a.add(ap().pose(5.2, 7.4).velocity(DEFAULT_TRENCH_VELOCITY).xap());
    a.add(parallel(stowIntake(), followPath("Starting Position Left Trench To Center")));
    a.add(parallel(deployIntake(), followPath("Left Side Conservative Sweep")));
    a.add(parallel(seq(waitSeconds(0.6), startShooting()), followPath("Left Bump To Alliance")));
    a.add(waitSeconds(0.1));
    a.add(ap().pose(LEFT_ALLIANCE_ZONE_MIDDLE_POSE).ap());
    a.add(waitSeconds(2.5));

    // wait(1.0),
    // Cycle 2
    if (doSecondSweep) {
      a.add(cycleIntake(intakeCycleTime, intakeCycleCount));
      a.add(
          ap().pose(3.5, 7.55, -90)
              .velocity(DEFAULT_TRENCH_VELOCITY)
              .entryAngle(0)
              .constraints(apConstraints(2.0, 2.0, 2.0))
              .pidGains(pidGains(1.5))
              .ap());
      a.add(parallel(stopShooting(), goToCenterUnderLeftTrenchFromAllianceIntakeIn()));
      a.add(parallel(deployIntake(), followPath("Left Side Close 2nd Sweep Intake In")));
      a.add(parallel(seq(waitSeconds(0.4), startShooting()), followPath("Left Bump To Alliance")));
      a.add(waitSeconds(0.1));
      a.add(ap().pose(2.700, 5.75, -90).ap());
      a.add(waitSeconds(1));
    }

    if (useDepot) {
      a.add(
          ap().pose(1.5, 5.9, -180)
              .velocity(0.0)
              .constraints(apConstraints(2.0, 2.0, 2.0))
              .pidGains(pidGains(1.5))
              .ap());
    }

    a.add(cycleIntake(intakeCycleTime, intakeCycleCount));
    return seq(a);
  }

  private static AutoAction follower() {
    List<AutoAction> f = new ArrayList<>();
    // Don't deploy the intake to avoid smashing it into the trench.
    // deployIntake();
    // startShooting();
    f.add(networkConfigurableWait("Follower - Preload", seconds(2.5)));
    // stopShooting();
    f.add(
        ap().pose(LEFT_TRENCH_CENTER_SIDE_POSE)
            .velocity(2.0)
            // Constraints here are unique to this very high speed, high
            // acceleration movement, so almost infinite acceleration limit is
            // hardcoded in here.
            .constraints(apConstraints(AGGRESSIVE_TRENCH_VELOCITY, 200.0))
            // No entry angle, we just want to beeline to trench exit.
            .xap());
    f.add(deployIntake());
    f.add(followPath("Left Side Follower Sweep Intake In"));
    f.add(networkConfigurableWait("Follower - Before Bump Return", seconds(0.0)));
    f.add(waitSeconds(0.1));
    f.add(ap().pose(2.700, 5.75, -90).ap());
    f.add(
        ap().pose(1.5, 5.1, 135)
            .profile(
                apProfile(apConstraints(5.1, 10.0, 3.0), meters(0.1), degrees(4.0), meters(0.2)))
            .ap());
    f.add(startShooting());
    f.add(networkConfigurableWait("Follower - Before Depot", seconds(0.0)));
    f.add(goToDepotAndIntake());
    f.add(waitSeconds(2.0));
    f.add(cycleIntake(6.0 / 3.0, 6));
    return seq(f);
  }

  /** Reusable climb lineup subroutine. */
  private static AutoAction climb(Pose2d targetPose, Rotation2d entryAngle) {
    return seq(
        parallel(
            seq(
                // xBasedAutopilot(FieldConstants.Alliance.center transformed by
                //   climb_offset, constraints=CLIMB_CONSTRAINTS)
                ap().pose(targetPose).entryAngle(entryAngle).constraints(CLIMB_CONSTRAINTS).xap()),
            climbSearch()),
        waitSeconds(0.5),
        climbHang());
  }

  private AutosGen() {}
}
