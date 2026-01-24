package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.littletonrobotics.junction.Logger;

class ShooterCalculations {
  private ShooterCalculations() {} // Utility class

  public enum ShotType {
    LOW,
    HIGH
  }

  public static record ShotInfo(double pitchRadians, double yawRadians, double timeSeconds) {
    public Translation3d[] projectMotion(
        double shooterVelocityMps,
        Translation3d initialPosition,
        ChassisSpeeds fieldRelativeRobotVel,
        double pointsPerMeter) {
      List<Translation3d> trajectory = new ArrayList<>();

      double vxy = shooterVelocityMps * Math.cos(pitchRadians());

      double vx = vxy * Math.cos(yawRadians()) + fieldRelativeRobotVel.vxMetersPerSecond;
      double vy = vxy * Math.sin(yawRadians()) + fieldRelativeRobotVel.vyMetersPerSecond;
      double vz = shooterVelocityMps * Math.sin(pitchRadians());

      Translation3d position = initialPosition;

      Function<Double, Translation3d> pointFromTime =
          t -> position.plus(new Translation3d(vx * t, vy * t, vz * t - 0.5 * G * t * t));
      Function<Double, Double> velocityFromTime = t -> new Vector3D(vx, vy, vz - G * t).getNorm();

      for (double t = 0; t < timeSeconds(); t += 1 / (velocityFromTime.apply(t) * pointsPerMeter)) {
        trajectory.add(pointFromTime.apply(t));
      }

      trajectory.add(pointFromTime.apply(timeSeconds()));

      return trajectory.toArray(new Translation3d[] {});
    }
  }

  public static final double G = 9.81;

  public static final double MIN_ALLOWED_DISTANCE = 1e-3;

  public static final double MIN_ALLOWED_COS = 1e-5;

  public static double calculateYawRadians(
      Translation3d shooterPosition, Translation3d goalPosition) {
    return Math.atan2(
        goalPosition.getY() - shooterPosition.getY(), goalPosition.getX() - shooterPosition.getX());
  }

  public static Optional<ShotInfo> calculateStationaryShot(
      Translation3d shooterPosition,
      Translation3d goalPosition,
      LinearVelocity shooterVelocity,
      ShotType shotType) {
    // Calculate horizontal distance
    var dvec = goalPosition.minus(shooterPosition);
    double distanceMeters = dvec.toTranslation2d().getNorm();

    double velocityMps = shooterVelocity.in(MetersPerSecond);

    if (distanceMeters < MIN_ALLOWED_DISTANCE || velocityMps <= 0.0) {
      return Optional.empty();
    }

    double h = dvec.getZ();
    double v2 = velocityMps * velocityMps;

    // Value inside the square root
    double discriminant = v2 * v2 - G * (G * distanceMeters * distanceMeters + 2 * h * v2);

    if (discriminant < 0.0) {
      return Optional.empty();
    }

    double sqrtDiscriminant = Math.sqrt(discriminant);

    double numerator = v2 + (shotType == ShotType.HIGH ? sqrtDiscriminant : -sqrtDiscriminant);

    double denominator = G * distanceMeters;

    double tanTheta = numerator / denominator;

    if (!Double.isFinite(tanTheta)) {
      return Optional.empty();
    }

    double theta = Math.atan(tanTheta);

    if (Math.cos(theta) < MIN_ALLOWED_COS) {
      return Optional.empty();
    }
    double t = distanceMeters / (Math.cos(theta) * velocityMps);
    // Here's an alternative calculation using height. It isn't quite right either.
    // double viy = velocityMps * Math.sin(theta);
    // double t = (viy + Math.sqrt(viy * viy - 2 * h * G)) / (G);

    double yaw = calculateYawRadians(shooterPosition, goalPosition);

    return Optional.of(new ShotInfo(theta, yaw, t));
  }

  public static final int MAX_ITERATIONS = 6;
  public static final double ACCEPTABLE_TIME_VARIATION = 0.025;

  public static Optional<ShotInfo> calculateMovingShot(
      Translation3d shooterPosition,
      Translation3d goalPosition,
      ChassisSpeeds robotVelocity,
      LinearVelocity shooterVelocity,
      ShotType shotType,
      Optional<ShotInfo> lastShot) {
    Logger.recordOutput("ShotCalculations/succeeded", false);
    Logger.recordOutput("ShotCalculations/goal", goalPosition);
    // Prepare a stationary shot to calculate if lastShot wasn't provided
    Optional<ShotInfo> baseShot =
        lastShot.or(
            () ->
                calculateStationaryShot(shooterPosition, goalPosition, shooterVelocity, shotType));

    if (baseShot.isEmpty()) {
      return Optional.empty();
    }

    // Consider adding an adjustment here based on how long it will take your turret/hood to
    // configure themselves for the shot.
    ShotInfo solution = baseShot.get();
    double t = solution.timeSeconds();

    int i;
    for (i = 0; i < MAX_ITERATIONS; i++) {
      Logger.recordOutput("ShotCalculations/Iterations", i + 1);
      t = solution.timeSeconds;

      var vRobot =
          new Translation3d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond, 0);

      Translation3d effectiveGoal = goalPosition.minus(vRobot.times(t));

      Logger.recordOutput("ShotCalculations/effectiveGoal", effectiveGoal);

      Optional<ShotInfo> effectiveShot =
          calculateStationaryShot(shooterPosition, effectiveGoal, shooterVelocity, shotType);

      if (effectiveShot.isEmpty()) {
        return Optional.empty();
      }

      ShotInfo newSolution = effectiveShot.get();

      if (Math.abs(newSolution.timeSeconds - solution.timeSeconds) < ACCEPTABLE_TIME_VARIATION) {
        Logger.recordOutput("ShotCalculations/succeeded", true);
        return Optional.of(newSolution);
      }

      solution = newSolution;
    }

    return Optional.empty();
  }
}
