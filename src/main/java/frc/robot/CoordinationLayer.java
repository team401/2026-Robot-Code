package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.ShooterCalculations.ShotInfo;
import frc.robot.ShooterCalculations.ShotType;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The coordination layer is responsible for updating subsystem dependencies, running the shot
 * calculator, and distributing commands to subsystems based on the action layer.
 */
public class CoordinationLayer {
  // Subsystems
  private final Optional<Drive> drive;
  private final Optional<TurretSubsystem> turret;
  private final Optional<HoodSubsystem> hood;
  private final Optional<Void> homingSwitch;

  // DOE Action Keys
  public static final ActionKey UPDATE_HOMING_SWITCH =
      new ActionKey("CoordinationLayer::updateHomingSwitch");
  public static final ActionKey UPDATE_TURRET_DEPENDENCIES =
      new ActionKey("CoordinationLayer::updateTurretDependencies");
  public static final ActionKey UPDATE_HOOD_DEPENDENCIES =
      new ActionKey("CoordinationLayer::updateHoodDependencies");
  public static final ActionKey RUN_SHOT_CALCULATOR =
      new ActionKey("CoordinationLayer::runShotCalculator");

  // State variables (these will be updated by various methods and then their values will be passed
  // to subsystems during the execution of a cycle)
  private boolean isHomingSwitchPressed = false;

  public CoordinationLayer(
      Optional<Drive> drive,
      Optional<TurretSubsystem> turret,
      Optional<HoodSubsystem> hood,
      Optional<Void> homingSwitch) {
    this.drive = drive;
    this.turret = turret;
    this.hood = hood;
    this.homingSwitch = homingSwitch;

    var dependencyOrderedExecutor = DependencyOrderedExecutor.getDefaultInstance();
    dependencyOrderedExecutor.registerAction(UPDATE_HOMING_SWITCH, this::updateHomingSwitch);
    dependencyOrderedExecutor.registerAction(
        UPDATE_TURRET_DEPENDENCIES, this::updateTurretDependencies);
    dependencyOrderedExecutor.registerAction(
        UPDATE_HOOD_DEPENDENCIES, this::updateHoodDependencies);
    dependencyOrderedExecutor.registerAction(RUN_SHOT_CALCULATOR, this::runShotCalculator);

    if (turret.isPresent()) {
      dependencyOrderedExecutor.addDependencies(RUN_SHOT_CALCULATOR, TurretSubsystem.UPDATE_INPUTS);
    }

    if (hood.isPresent()) {
      dependencyOrderedExecutor.addDependencies(RUN_SHOT_CALCULATOR, HoodSubsystem.UPDATE_INPUTS);
    }

    dependencyOrderedExecutor.addDependencies(UPDATE_TURRET_DEPENDENCIES, UPDATE_HOMING_SWITCH);
    dependencyOrderedExecutor.addDependencies(UPDATE_HOOD_DEPENDENCIES, UPDATE_HOMING_SWITCH);
  }

  // Subsystem dependency updates
  /**
   * Update the CoordinationLayer homing switch stated based on the last value from the homing
   * switch
   */
  private void updateHomingSwitch() {
    // TODO: Add actual check for whether homing switch is pressed once it is designed
    isHomingSwitchPressed = homingSwitch.map(s -> false).orElse(false);
  }

  /** Update the turret subsystem on the state of the homing switch. */
  private void updateTurretDependencies() {
    turret.ifPresent(turret -> turret.setIsHomingSwitchPressed(isHomingSwitchPressed));
  }

  /** Update the hood subsystem on the state of the homing switch. */
  private void updateHoodDependencies() {
    hood.ifPresent(hood -> hood.setIsHomingSwitchPressed(isHomingSwitchPressed));
  }

  // Coordination and processing
  /**
   * Runs the shot calculator and logs the resulting trajectories for debugging. Eventually, this
   * method should also command the subsystems to take their actinos.
   *
   * <p>This should likely go into some kind of coordination layer class.
   */
  public void runShotCalculator() {
    drive.ifPresent(
        driveInstance -> {
          Translation3d hubTranslation =
              new Translation3d(
                  Units.inchesToMeters(182.11),
                  Units.inchesToMeters(158.84),
                  Units.inchesToMeters(72 - 8));
          // Placeholder passing "example" translation
          Translation3d passingTargetTranslation = new Translation3d(2.0, 1.0, 0.0);

          Pose2d robotPose = driveInstance.getPose();
          // Pose taken from CAD
          Transform3d robotToShooter =
              new Transform3d(
                  Units.inchesToMeters(4.175),
                  Units.inchesToMeters(-2.088),
                  Units.inchesToMeters(17.0),
                  new Rotation3d());
          Translation3d shooterPosition =
              new Pose3d(robotPose).plus(robotToShooter).getTranslation();

          // Pick either passing target or hub here.
          Translation3d goalTranslation = hubTranslation;

          ChassisSpeeds fieldCentricSpeeds =
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  driveInstance.getChassisSpeeds(), robotPose.getRotation());

          Translation2d goalXYPlane =
              new Translation2d(goalTranslation.getX(), goalTranslation.getY());
          double shotDistanceXYMeters = robotPose.getTranslation().getDistance(goalXYPlane);
          Logger.recordOutput("CoordinationLayer/ShotXYDistanceMeters", shotDistanceXYMeters);

          double viMetersPerSecond =
              JsonConstants.shooterConstants.getViFromDistance(shotDistanceXYMeters);
          Logger.recordOutput("CoordinationLayer/viMetersPerSecond", viMetersPerSecond);

          Optional<ShotInfo> maybeShot =
              ShooterCalculations.calculateMovingShot(
                  shooterPosition,
                  goalTranslation,
                  fieldCentricSpeeds,
                  MetersPerSecond.of(viMetersPerSecond),
                  ShotType.HIGH,
                  Optional.empty());

          Optional<ShotInfo> maybeStaticShot =
              ShooterCalculations.calculateStationaryShot(
                  shooterPosition,
                  goalTranslation,
                  MetersPerSecond.of(viMetersPerSecond),
                  ShotType.HIGH);

          final int trajectoryPointsPerMeter = 4;

          maybeShot.ifPresent(
              shot -> {
                shot =
                    new ShotInfo(
                        MathUtil.clamp(
                            shot.pitchRadians(), Math.toRadians(90 - 40), Math.toRadians(90 - 20)),
                        shot.yawRadians(),
                        shot.timeSeconds());
                Translation3d[] shotTrajectory =
                    shot.projectMotion(
                        viMetersPerSecond,
                        shooterPosition,
                        fieldCentricSpeeds,
                        trajectoryPointsPerMeter);
                Logger.recordOutput("CoordinationLayer/Shot", shot);
                Logger.recordOutput(
                    "CoordinationLayer/EffectiveTrajectory",
                    shot.projectMotion(
                        viMetersPerSecond,
                        shooterPosition,
                        new ChassisSpeeds(),
                        trajectoryPointsPerMeter));
                Logger.recordOutput("CoordinationLayer/ShotTrajectory", shotTrajectory);
                Logger.recordOutput(
                    "CoordinationLayer/ShotErrorMeters",
                    shotTrajectory[shotTrajectory.length - 1].getDistance(goalTranslation));
              });

          maybeStaticShot.ifPresent(
              shot -> {
                Logger.recordOutput(
                    "CoordinationLayer/StaticTrajectory",
                    shot.projectMotion(
                        viMetersPerSecond,
                        shooterPosition,
                        fieldCentricSpeeds,
                        trajectoryPointsPerMeter));
              });
        });
  }
}
