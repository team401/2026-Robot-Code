package frc.robot.autogen;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import coppercore.wpilib_interface.tuning.PIDGains;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.auto.AutoAction;
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
import frc.robot.auto.general.NetworkConfigurableWait;
import frc.robot.auto.general.Parallel;
import frc.robot.auto.general.Print;
import frc.robot.auto.general.Race;
import frc.robot.auto.general.Sequence;
import frc.robot.auto.general.Wait;
import java.util.List;

/**
 * Small authoring DSL for building auto routines in Java. Mirrors the helpers that used to live in
 * the Python {@code shorthands.py}/{@code auto_lib.py}. All factory methods return plain {@link
 * AutoAction} objects; nesting is expressed with {@link #seq}, {@link #parallel}, {@link #race} and
 * {@link #deadline}.
 */
public final class Dsl {
  private Dsl() {}

  // ---------------------------------------------------------------------------
  // Geometry helpers
  // ---------------------------------------------------------------------------

  public static Rotation2d rotation2d(double degrees) {
    return Rotation2d.fromDegrees(degrees);
  }

  public static Translation2d translation2d(double x, double y) {
    return new Translation2d(x, y);
  }

  public static Pose2d pose2d(double x, double y, double angleDegrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
  }

  public static Pose2d pose2d(double x, double y) {
    return pose2d(x, y, 0.0);
  }

  public static Transform2d transform2d(Translation2d translation, Rotation2d rotation) {
    return new Transform2d(translation, rotation);
  }

  // ---------------------------------------------------------------------------
  // Units
  // ---------------------------------------------------------------------------

  public static Time seconds(double value) {
    return Seconds.of(value);
  }

  public static Distance meters(double value) {
    return Meters.of(value);
  }

  public static Angle degrees(double value) {
    return Degrees.of(value);
  }

  // ---------------------------------------------------------------------------
  // Autopilot building blocks
  // ---------------------------------------------------------------------------

  public static APConstraints apConstraints(double velocity, double acceleration, double jerk) {
    return new APConstraints(velocity, acceleration, jerk);
  }

  public static APConstraints apConstraints(double velocity, double acceleration) {
    // NB: the autopilot library's 2-arg constructor means (acceleration, jerk) with velocity
    // unlimited. The Python generator's APConstraints(v, a) meant (velocity, acceleration) with
    // jerk = 0, so build that explicitly via the 3-arg constructor.
    return new APConstraints(velocity, acceleration, 0.0);
  }

  public static APProfile apProfile(
      APConstraints constraints, Distance errorXY, Angle errorTheta, Distance beelineRadius) {
    return new APProfile(constraints)
        .withErrorXY(errorXY)
        .withErrorTheta(errorTheta)
        .withBeelineRadius(beelineRadius);
  }

  public static PIDGains pidGains(double kP) {
    return PIDGains.kPID(kP, 0.0, 0.0);
  }

  /** Fluent builder for {@link AutoPilotAction} / {@link XBasedAutoPilotAction}. */
  public static final class Ap {
    private Pose2d reference;
    private Rotation2d entryAngle;
    private double velocity = 0.0;
    private Distance rotationRadius;
    private APConstraints constraints;
    private APProfile profile;
    private PIDGains pidGains;
    private boolean canMirror = true;

    public Ap pose(double x, double y, double angleDegrees) {
      this.reference = pose2d(x, y, angleDegrees);
      return this;
    }

    public Ap pose(double x, double y) {
      return pose(x, y, 0.0);
    }

    public Ap pose(Pose2d reference) {
      this.reference = reference;
      return this;
    }

    public Ap velocity(double velocity) {
      this.velocity = velocity;
      return this;
    }

    public Ap entryAngle(double degrees) {
      this.entryAngle = rotation2d(degrees);
      return this;
    }

    public Ap entryAngle(Rotation2d entryAngle) {
      this.entryAngle = entryAngle;
      return this;
    }

    public Ap rotationRadius(Distance rotationRadius) {
      this.rotationRadius = rotationRadius;
      return this;
    }

    public Ap constraints(APConstraints constraints) {
      this.constraints = constraints;
      return this;
    }

    public Ap profile(APProfile profile) {
      this.profile = profile;
      return this;
    }

    public Ap pidGains(PIDGains pidGains) {
      this.pidGains = pidGains;
      return this;
    }

    public Ap canMirror(boolean canMirror) {
      this.canMirror = canMirror;
      return this;
    }

    private APTarget target() {
      APTarget target = new APTarget(reference == null ? new Pose2d() : reference);
      if (entryAngle != null) {
        target = target.withEntryAngle(entryAngle);
      }
      target = target.withVelocity(velocity);
      if (rotationRadius != null) {
        target = target.withRotationRadius(rotationRadius);
      }
      return target;
    }

    public AutoPilotAction ap() {
      return new AutoPilotAction(target(), constraints, profile, pidGains, canMirror);
    }

    public XBasedAutoPilotAction xap() {
      return new XBasedAutoPilotAction(target(), constraints, profile, pidGains, canMirror);
    }
  }

  public static Ap ap() {
    return new Ap();
  }

  // ---------------------------------------------------------------------------
  // Primitive action shorthands
  // ---------------------------------------------------------------------------

  public static AutoAction startShooting() {
    return new StartShooting();
  }

  public static AutoAction stopShooting() {
    return new StopShooting();
  }

  public static AutoAction deployIntake() {
    return new DeployIntakeAction();
  }

  public static AutoAction stowIntake() {
    return new StowIntakeAction();
  }

  public static AutoAction climbSearch() {
    return new ClimbSearchAction();
  }

  public static AutoAction climbHang() {
    return new ClimbHangAction();
  }

  public static AutoAction stopDrive() {
    return new StopDriveAction();
  }

  public static AutoAction waitSeconds(double secondsValue) {
    Wait wait = new Wait();
    wait.delay = seconds(secondsValue);
    return wait;
  }

  public static AutoAction print(String message) {
    Print print = new Print();
    print.message = message;
    return print;
  }

  public static AutoAction reference(String autoName) {
    AutoReference ref = new AutoReference();
    ref.name = autoName;
    return ref;
  }

  public static AutoAction networkConfigurableWait(String name, Time defaultDelay) {
    return new NetworkConfigurableWait(name, defaultDelay);
  }

  public static AutoAction followPath(String pathName) {
    return followPath(pathName, false, true);
  }

  public static AutoAction followPath(String pathName, boolean mirrorPath, boolean canMirror) {
    FollowPathPlannerPath path = new FollowPathPlannerPath();
    path.pathName = pathName;
    path.mirrorPath = mirrorPath;
    path.canMirror = canMirror;
    return path;
  }

  // ---------------------------------------------------------------------------
  // Containers
  // ---------------------------------------------------------------------------

  public static Sequence seq(AutoAction... actions) {
    Sequence sequence = new Sequence();
    sequence.actions = actions;
    return sequence;
  }

  public static Sequence seq(List<AutoAction> actions) {
    return seq(actions.toArray(new AutoAction[0]));
  }

  public static Parallel parallel(AutoAction... actions) {
    Parallel parallel = new Parallel();
    parallel.actions = actions;
    return parallel;
  }

  public static Parallel parallel(List<AutoAction> actions) {
    return parallel(actions.toArray(new AutoAction[0]));
  }

  public static Race race(AutoAction... actions) {
    Race race = new Race();
    race.actions = actions;
    return race;
  }
}
