package frc.robot.util.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// For now mostly just a copy and paste, but will edit more later

// Mostly Copied from com.ctre.phoenix6.swerve.utility.LinearPath

// TODO: Figure out how to make it care about direction of speed at the end of the path.

public class AdjustableLinearPath {
  public static class State {
    public Pose2d pose;
    public ChassisSpeeds speeds;

    public State() {
      this(Pose2d.kZero, new ChassisSpeeds());
    }

    public State(Pose2d pose, ChassisSpeeds speeds) {
      this.pose = pose;
      this.speeds = speeds;
    }
  }

  private TrapezoidProfile linearProfile;
  private TrapezoidProfile angularProfile;

  private Pose2d initialPose = Pose2d.kZero;
  private Rotation2d heading = Rotation2d.kZero;

  private TrapezoidProfile.State linearGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State linearStart = new TrapezoidProfile.State();

  private TrapezoidProfile.State angularGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State angularStart = new TrapezoidProfile.State();

  public AdjustableLinearPath(
      TrapezoidProfile.Constraints linearConstraints,
      TrapezoidProfile.Constraints angularConstraints) {
    this.linearProfile = new TrapezoidProfile(linearConstraints);
    this.angularProfile = new TrapezoidProfile(angularConstraints);
  }

  public void setConstraints(
      TrapezoidProfile.Constraints linearConstraints,
      TrapezoidProfile.Constraints angularConstraints) {
    this.linearProfile = new TrapezoidProfile(linearConstraints);
    this.angularProfile = new TrapezoidProfile(angularConstraints);
  }

  public void setLinearConstraints(TrapezoidProfile.Constraints linearConstraints) {
    this.linearProfile = new TrapezoidProfile(linearConstraints);
  }

  public void setAngularConstraints(TrapezoidProfile.Constraints angularConstraints) {
    this.angularProfile = new TrapezoidProfile(angularConstraints);
  }

  private static double calculateVelocityAtHeading(ChassisSpeeds speeds, Rotation2d heading) {
    // vel = <vx, vy> ⋅ <cos(heading), sin(heading)>
    // vel = vx * cos(heading) + vy * sin(heading)
    return speeds.vxMetersPerSecond * heading.getCos()
        + speeds.vyMetersPerSecond * heading.getSin();
  }

  private void setState(State current, State goal) {
    this.initialPose = current.pose;

    {
      // pull out the translation from our initial pose to the target
      final var translation = goal.pose.getTranslation().minus(this.initialPose.getTranslation());
      // pull out distance and heading to the target
      final var distance = translation.getNorm();
      if (distance > 1e-6) {
        this.heading = translation.getAngle();
      } else {
        this.heading = Rotation2d.kZero;
      }

      final var linearGoalSpeed = calculateVelocityAtHeading(goal.speeds, this.heading);

      this.linearGoal = new TrapezoidProfile.State(distance, linearGoalSpeed);
    }

    {
      // start at current velocity in the direction of travel
      final var vel = calculateVelocityAtHeading(current.speeds, this.heading);
      this.linearStart = new TrapezoidProfile.State(0, vel);
    }

    this.angularStart =
        new TrapezoidProfile.State(
            current.pose.getRotation().getRadians(), current.speeds.omegaRadiansPerSecond);
    // wrap the angular goal so we take the shortest path
    this.angularGoal =
        new TrapezoidProfile.State(
            current.pose.getRotation().getRadians()
                + goal.pose.getRotation().minus(current.pose.getRotation()).getRadians(),
            goal.speeds.omegaRadiansPerSecond);
  }

  private State calculate(double t) {
    // calculate our new distance and velocity in the desired direction of travel
    final var linearState = this.linearProfile.calculate(t, this.linearStart, this.linearGoal);
    // calculate our new heading and rotational rate
    final var angularState = this.angularProfile.calculate(t, this.angularStart, this.angularGoal);

    // x is m_state * cos(heading), y is m_state * sin(heading)
    final var pose =
        new Pose2d(
            this.initialPose.getX() + linearState.position * this.heading.getCos(),
            this.initialPose.getY() + linearState.position * this.heading.getSin(),
            Rotation2d.fromRadians(angularState.position));
    final var speeds =
        new ChassisSpeeds(
            linearState.velocity * this.heading.getCos(),
            linearState.velocity * this.heading.getSin(),
            angularState.velocity);
    return new State(pose, speeds);
  }

  public final State calculate(double t, State current, State goal) {
    setState(current, goal);
    return calculate(t);
  }

  public final double totalTime() {
    return Math.max(this.linearProfile.totalTime(), this.angularProfile.totalTime());
  }

  public final boolean isFinished(double t) {
    return t >= totalTime();
  }
}
