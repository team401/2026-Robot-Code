package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.AllianceBasedFieldConstants;
import frc.robot.constants.FieldLocations;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ShotMaps.ShotMap;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.littletonrobotics.junction.Logger;

class ShotCalculations {
  private ShotCalculations() {} // Utility class

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

  public static enum ShotTarget {
    Hub,
    PassLeft,
    PassRight
  }

  /**
   * The MapBasedShotInfo record provides information about a shot based on a shooter map. This is
   * different from ShotInfo in that MapBasedShotInfo has fields that relate to physical control of
   * the mechanism (e.g. hood angle and shooter RPM) rather than physics-based attributes of the
   * goal shot (e.g. shot pitch and initial velocity).
   *
   * @param hoodAngleRadians The angle of the hood (NOT the pitch) in radians.
   * @param shooterRPM The desired angular velocity of the shooter, in RPM.
   * @param yaw The desired field-relative yaw (NOT the angle setpoint) of the turret, in a
   *     Rotation2d.
   * @param isReal {@code true} if the shot is a real shot aimed at the target and {@code false} if
   *     the shot is a "best guess" because the real shot is impossible.
   */
  public record MapBasedShotInfo(
      double hoodAngleRadians, double shooterRPM, Rotation2d yaw, boolean isReal) {}

  /**
   * Calculate a MapBasedShotInfo by looking ahead from the current position and accounting for
   * motion using time of flight.
   *
   * @param robotPose The current robot position
   * @param robotRelativeChassisSpeeds The ROBOT CENTRIC chassis speeds of the robot
   * @param fieldRelativeShooterVelocity the velocity of the shooter, field relative, as a
   *     Translation2d.
   * @param target the ShotTarget to aim for
   * @return
   */
  public static MapBasedShotInfo calculateShotFromMap(
      Pose2d robotPose,
      ChassisSpeeds robotRelativeChassisSpeeds,
      Translation2d fieldRelativeShooterVelocity,
      ShotTarget target) {
    Translation2d targetPosition =
        switch (target) {
          case Hub -> AllianceBasedFieldConstants.hubCenterPoint2d();
          case PassLeft -> FieldLocations.leftPassingTarget();
          case PassRight -> FieldLocations.rightPassingTarget();
        };

    double lookaheadTimeSeconds = JsonConstants.shotMaps.mechanismCompensationDelay.in(Seconds);

    Pose2d lookaheadPose =
        robotPose.exp(
            new Twist2d(
                robotRelativeChassisSpeeds.vxMetersPerSecond * lookaheadTimeSeconds,
                robotRelativeChassisSpeeds.vyMetersPerSecond * lookaheadTimeSeconds,
                robotRelativeChassisSpeeds.omegaRadiansPerSecond * lookaheadTimeSeconds));

    Logger.recordOutput("ShotCalculations/MapBased/lookaheadPose", lookaheadPose);

    Pose2d shooterPose = lookaheadPose.plus(JsonConstants.robotInfo.robotToShooter2d);

    Logger.recordOutput("ShotCalculations/MapBased/shooterPose", shooterPose);

    double distanceXYMeters = shooterPose.getTranslation().getDistance(targetPosition);
    Logger.recordOutput("ShotCalculations/MapBased/ShotDistanceMeters", distanceXYMeters);

    double minDistanceMeters =
        target == ShotTarget.Hub
            ? JsonConstants.shotMaps.minHubDistanceMeters
            : JsonConstants.shotMaps.minPassDistanceMeters;
    double maxDistanceMeters =
        target == ShotTarget.Hub
            ? JsonConstants.shotMaps.maxHubDistanceMeters
            : JsonConstants.shotMaps.maxPassDistanceMeters;

    Logger.recordOutput("ShotCalculations/MapBased/MinDistanceMeters", minDistanceMeters);
    Logger.recordOutput("ShotCalculations/MapBased/MaxDistanceMeters", maxDistanceMeters);

    // Track whether or not the shot has been clamped to make it fit into the distance map.
    // If it hasn't been clamped, it is no longer real: it is a best approximation of the shot we
    // will take soon.
    boolean isShotReal = true;

    if (distanceXYMeters < minDistanceMeters || distanceXYMeters > maxDistanceMeters) {
      // When clamping, the shot is no longer real.
      isShotReal = false;
      distanceXYMeters = MathUtil.clamp(distanceXYMeters, minDistanceMeters, maxDistanceMeters);
    }
    Logger.recordOutput("ShotCalculations/MapBased/ClampedShotDistanceMeters", distanceXYMeters);

    ShotMap map =
        target == ShotTarget.Hub
            ? JsonConstants.shotMaps.hubMap
            : JsonConstants.shotMaps.passingMap;

    double flightTimeSeconds = map.flightTimeSecondsByDistanceMeters().get(distanceXYMeters);
    Logger.recordOutput("ShotCalculations/MapBased/FlightTimeSeconds", flightTimeSeconds);

    Translation2d virtualTarget = new Translation2d();

    double virtualDistanceXYMeters = 0.0;
    for (int i = 0; i < MAX_ITERATIONS; i++) {
      Translation2d offset = fieldRelativeShooterVelocity.times(flightTimeSeconds);
      virtualTarget = targetPosition.minus(offset);

      virtualDistanceXYMeters = shooterPose.getTranslation().getDistance(virtualTarget);

      double lastFlightTimeSeconds = flightTimeSeconds;
      flightTimeSeconds = map.flightTimeSecondsByDistanceMeters().get(virtualDistanceXYMeters);
      if (Math.abs(flightTimeSeconds - lastFlightTimeSeconds) < ACCEPTABLE_TIME_VARIATION) {
        break;
      }
    }
    Logger.recordOutput("ShotCalculations/MapBased/VirtualTarget", virtualTarget);
    Logger.recordOutput("ShotCalculations/MapBased/VirtualDistanceMeters", virtualDistanceXYMeters);

    if (virtualDistanceXYMeters < minDistanceMeters
        || virtualDistanceXYMeters > maxDistanceMeters) {
      // When clamping, the shot is no longer real.
      isShotReal = false;
      virtualDistanceXYMeters =
          MathUtil.clamp(virtualDistanceXYMeters, minDistanceMeters, maxDistanceMeters);
    }
    Logger.recordOutput(
        "ShotCalculations/MapBased/ClampedVirtualDistanceMeters", virtualDistanceXYMeters);

    double hoodAngleRadians = map.hoodAngleRadiansByDistanceMeters().get(virtualDistanceXYMeters);
    double shooterRPM = map.rpmByDistanceMeters().get(virtualDistanceXYMeters);
    Rotation2d yawRadians = virtualTarget.minus(shooterPose.getTranslation()).getAngle();

    MapBasedShotInfo shot =
        new MapBasedShotInfo(hoodAngleRadians, shooterRPM, yawRadians, isShotReal);
    Logger.recordOutput("ShotCalculations/MapBased/Shot", shot);
    return shot;
  }
}
