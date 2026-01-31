package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.ShotCalculations.ShotInfo;
import frc.robot.ShotCalculations.ShotType;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.HomingSwitch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The coordination layer is responsible for updating subsystem dependencies, running the shot
 * calculator, and distributing commands to subsystems based on the action layer.
 */
public class CoordinationLayer {
  // Subsystems
  private Optional<Drive> drive = Optional.empty();
  private Optional<HopperSubsystem> hopper = Optional.empty();
  private Optional<IndexerSubsystem> indexer = Optional.empty();
  private Optional<TurretSubsystem> turret = Optional.empty();
  private Optional<HoodSubsystem> hood = Optional.empty();
  // The homing switch will likely be either added to one subsystem or made its own subsystem later
  private Optional<HomingSwitch> homingSwitch = Optional.empty();

  private final DependencyOrderedExecutor dependencyOrderedExecutor;

  // DOE Action Keys
  public static final ActionKey UPDATE_TURRET_DEPENDENCIES =
      new ActionKey("CoordinationLayer::updateTurretDependencies");
  public static final ActionKey UPDATE_HOOD_DEPENDENCIES =
      new ActionKey("CoordinationLayer::updateHoodDependencies");
  public static final ActionKey RUN_SHOT_CALCULATOR =
      new ActionKey("CoordinationLayer::runShotCalculator");
  public static final ActionKey RUN_DEMO_MODES =
      new ActionKey("CoordinationLayer::runSubsystemDemoModes");

  // State variables (these will be updated by various methods and then their values will be passed
  // to subsystems during the execution of a cycle)
  public CoordinationLayer(DependencyOrderedExecutor dependencyOrderedExecutor) {
    this.dependencyOrderedExecutor = dependencyOrderedExecutor;

    dependencyOrderedExecutor.registerAction(RUN_SHOT_CALCULATOR, this::runShotCalculator);
    dependencyOrderedExecutor.registerAction(RUN_DEMO_MODES, this::runSubsystemDemoModes);
  }

  /**
   * Checks whether a subsystem has already been initialized and, if it has, throws an error.
   *
   * @param optionalSubsystem The Optional potentially containing the already-initialized subsystem
   * @param name The name of the subsystem, capitalized, as it appears in the set... method, to use
   *     in the error message (e.g. "Hopper" for "setHopper")
   */
  private void checkForDuplicateSubsystem(Optional<?> optionalSubsystem, String name) {
    if (optionalSubsystem.isPresent()) {
      throw new IllegalStateException("CoordinationLayer set" + name + " was called twice!");
    }
  }

  public void setDrive(Drive drive) {
    checkForDuplicateSubsystem(this.drive, "Drive");
    this.drive = Optional.of(drive);
  }

  public void setHopper(HopperSubsystem hopper) {
    checkForDuplicateSubsystem(this.hopper, "Hopper");
    this.hopper = Optional.of(hopper);
  }

  public void setIndexer(IndexerSubsystem indexer) {
    checkForDuplicateSubsystem(this.indexer, "Indexer");
    this.indexer = Optional.of(indexer);
  }

  /**
   * Sets the turret instance for the CoordinationLayer to use.
   *
   * <p>This method must run before the DependencyOrderedExecutor's schedule is finalized, as it
   * adds dependencies. However, if the turret subsystem is disabled in FeatureFlags, it does not
   * need to be called at all.
   *
   * @param turret The TurretSubsystem instance to use
   */
  public void setTurret(TurretSubsystem turret) {
    checkForDuplicateSubsystem(this.turret, "Turret");
    this.turret = Optional.of(turret);

    dependencyOrderedExecutor.registerAction(
        UPDATE_TURRET_DEPENDENCIES, () -> updateTurretDependencies(turret));

    dependencyOrderedExecutor.addDependencies(RUN_SHOT_CALCULATOR, TurretSubsystem.UPDATE_INPUTS);
  }

  /**
   * Sets the hood instance for the CoordinationLayer to use.
   *
   * <p>This method must run before the DependencyOrderedExecutor's schedule is finalized, as it
   * adds dependencies. However, if the hood subsystem is disabled in FeatureFlags, it does not need
   * to be called at all.
   *
   * @param hood The HoodSubsystem instance to use
   */
  public void setHood(HoodSubsystem hood) {
    if (this.hood.isPresent()) {
      throw new IllegalStateException("CoordinationLayer setHood was called twice!");
    }

    this.hood = Optional.of(hood);

    dependencyOrderedExecutor.registerAction(
        UPDATE_HOOD_DEPENDENCIES, () -> updateHoodDependencies(hood));

    dependencyOrderedExecutor.addDependencies(RUN_SHOT_CALCULATOR, HoodSubsystem.UPDATE_INPUTS);
  }

  public void setHomingSwitch(HomingSwitch homingSwitch) {
    if (this.homingSwitch.isPresent()) {
      throw new IllegalStateException("CoordinationLayer setHomingSwitch was called twice!");
    }

    this.homingSwitch = Optional.of(homingSwitch);

    if (JsonConstants.featureFlags.runTurret) {
      dependencyOrderedExecutor.addDependencies(
          UPDATE_TURRET_DEPENDENCIES, HomingSwitch.UPDATE_INPUTS);
    }
    if (JsonConstants.featureFlags.runHood) {
      dependencyOrderedExecutor.addDependencies(
          UPDATE_HOOD_DEPENDENCIES, HomingSwitch.UPDATE_INPUTS);
    }
  }

  private boolean isHomingSwitchPressed() {
    return homingSwitch.map(homingSwitch -> homingSwitch.isHomingSwitchPressed()).orElse(false);
  }

  // Subsystem dependency updates
  /** Update the turret subsystem on the state of the homing switch. */
  private void updateTurretDependencies(TurretSubsystem turret) {
    turret.setIsHomingSwitchPressed(isHomingSwitchPressed());
    drive.ifPresent(
        drive -> {
          turret.setRobotHeading(drive.getRotation());
        });
  }

  /** Update the hood subsystem on the state of the homing switch. */
  private void updateHoodDependencies(HoodSubsystem hood) {
    hood.setIsHomingSwitchPressed(isHomingSwitchPressed());
  }

  // Coordination and processing
  /**
   * Runs the shot calculator and logs the resulting trajectories for debugging. Eventually, this
   * method should also command the subsystems to take their actinos.
   *
   * <p>This should likely go into some kind of coordination layer class.
   */
  public void runShotCalculator() {
    drive.ifPresent(this::runShotCalculatorWithDrive);
  }

  /**
   * Run the shot calculations, given an actual drive instance
   *
   * <p>This method exists to reduce the indentation in runShotCalculator introduced by widespread
   * use of optionals
   *
   * @param drive A Drive instance
   */
  private void runShotCalculatorWithDrive(Drive driveInstance) {
    Translation3d hubTranslation = FieldConstants.Hub.innerCenterPoint();
    // Placeholder passing "example" translation
    Translation3d passingTargetTranslation = new Translation3d(2.0, 1.0, 0.0);

    Pose2d robotPose = driveInstance.getPose();
    Translation3d shooterPosition =
        new Pose3d(robotPose).plus(JsonConstants.robotInfo.robotToShooter).getTranslation();

    // Pick either passing target or hub here.
    Translation3d goalTranslation = hubTranslation;

    ChassisSpeeds fieldCentricSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            driveInstance.getChassisSpeeds(), robotPose.getRotation());

    Translation2d goalXYPlane = new Translation2d(goalTranslation.getX(), goalTranslation.getY());
    double shotDistanceXYMeters = robotPose.getTranslation().getDistance(goalXYPlane);
    Logger.recordOutput("CoordinationLayer/ShotXYDistanceMeters", shotDistanceXYMeters);

    double viMetersPerSecond =
        JsonConstants.shooterConstants.getViFromDistance(shotDistanceXYMeters);
    Logger.recordOutput("CoordinationLayer/viMetersPerSecond", viMetersPerSecond);

    Optional<ShotInfo> maybeShot =
        ShotCalculations.calculateMovingShot(
            shooterPosition,
            goalTranslation,
            fieldCentricSpeeds,
            MetersPerSecond.of(viMetersPerSecond),
            ShotType.HIGH,
            Optional.empty());

    maybeShot.ifPresent(
        idealShot -> {
          sendIdealShotToSubsystems(idealShot);

          final int trajectoryPointsPerMeter = 4;

          ShotInfo currentShot = getCurrentShot(idealShot);

          Translation3d[] idealShotTrajectory =
              idealShot.projectMotion(
                  viMetersPerSecond, shooterPosition, fieldCentricSpeeds, trajectoryPointsPerMeter);
          Translation3d[] shotTrajectory =
              currentShot.projectMotion(
                  viMetersPerSecond, shooterPosition, fieldCentricSpeeds, trajectoryPointsPerMeter);
          Logger.recordOutput("CoordinationLayer/IdealShot", idealShot);
          Logger.recordOutput("CoordinationLayer/Shot", currentShot);

          // Logger.recordOutput(
          //     "CoordinationLayer/EffectiveTrajectory",
          //     idealShot.projectMotion(
          //         viMetersPerSecond,
          //         shooterPosition,
          //         new ChassisSpeeds(),
          //         trajectoryPointsPerMeter));
          Logger.recordOutput("CoordinationLayer/ShotTrajectory", shotTrajectory);
          Logger.recordOutput("CoordinationLayer/IdealShotTrajectory", idealShotTrajectory);
          Logger.recordOutput(
              "CoordinationLayer/ShotErrorMeters",
              shotTrajectory[shotTrajectory.length - 1].getDistance(goalTranslation));
        });
  }

  private void runSubsystemDemoModes() {
    if (JsonConstants.hopperConstants.hopperDemoMode) {
      hopper.ifPresent(
          hopper -> {
            hopper.setTargetVelocity(RadiansPerSecond.of(500));
          });
    }

    if (JsonConstants.indexerConstants.indexerDemoMode) {
      indexer.ifPresent(
          indexer -> {
            drive.ifPresent(
                driveInstance -> {
                  Pose2d robotPose = driveInstance.getPose();
                  Translation2d hubTranslation =
                      FieldConstants.Hub.topCenterPoint().toTranslation2d();
                  var distance = robotPose.getTranslation().minus(hubTranslation).getNorm();
                  if (distance < 2.0) {
                    indexer.setTargetVelocity(RadiansPerSecond.of(500));
                  } else {
                    indexer.setTargetVelocity(RadiansPerSecond.of(0));
                  }
                });
          });
    }
    ;
  }

  /**
   * Given an "ideal" shot, command the scoring subsystems to target it
   *
   * <p>This method exists to reduce the indentation introduced by the use of Optionals in
   * runShotCalculator
   *
   * @param idealShot A ShotInfo representing the ideal shot to take
   */
  private void sendIdealShotToSubsystems(ShotInfo idealShot) {
    // Command subsystems to follow ideal shot
    turret.ifPresent(
        turret -> {
          turret.targetGoalHeading(new Rotation2d(idealShot.yawRadians()));
        });
    hood.ifPresent(
        hood -> {
          hood.targetPitch(Radians.of(idealShot.pitchRadians()));
        });
  }

  /**
   * Get the "current shot" that the robot is aimed for.
   *
   * @param idealShot The "ideal shot" to use for fallback values if certain subsystems are disabled
   * @return A ShotInfo representing where the robot is currently aimed
   */
  private ShotInfo getCurrentShot(ShotInfo idealShot) {
    return new ShotInfo(
        hood.map(hood -> hood.getCurrentExitAngle().in(Radians))
            .orElse(
                MathUtil.clamp(
                    idealShot.pitchRadians(), Math.toRadians(90 - 40), Math.toRadians(90 - 20))),
        turret
            .map(turret -> turret.getFieldCentricTurretHeading().getRadians())
            .orElse(idealShot.yawRadians()),
        idealShot.timeSeconds());
  }
}
