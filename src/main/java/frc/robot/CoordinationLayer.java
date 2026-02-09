package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.ShotCalculations.MapBasedShotInfo;
import frc.robot.ShotCalculations.ShotInfo;
import frc.robot.ShotCalculations.ShotTarget;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.HomingSwitch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Optional;

/**
 * The coordination layer is responsible for updating subsystem dependencies, running the shot
 * calculator, and distributing commands to subsystems based on the action layer.
 */
public class CoordinationLayer {
  // Subsystems
  private Optional<Drive> drive = Optional.empty();
  private Optional<DriveCoordinator> driveCoordinator = Optional.empty();
  private Optional<HopperSubsystem> hopper = Optional.empty();
  private Optional<IndexerSubsystem> indexer = Optional.empty();
  private Optional<TurretSubsystem> turret = Optional.empty();
  private Optional<ShooterSubsystem> shooter = Optional.empty();
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

  public void setDriveCoordinator(DriveCoordinator driveCoordinator) {
    checkForDuplicateSubsystem(this.driveCoordinator, "DriveCoordinator");
    this.driveCoordinator = Optional.of(driveCoordinator);
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
   * Sets the shooter instance for the CoordinationLayer to use.
   *
   * <p>This method must run before the DependencyOrderedExecutor's schedule is finalized, as it
   * adds dependencies. However, if the shooter subsystem is disabled in FeatureFlags, it does not
   * need to be called at all.
   *
   * @param shooter The ShooterSubsystem instance to use
   */
  public void setShooter(ShooterSubsystem shooter) {
    checkForDuplicateSubsystem(this.shooter, "Shooter");

    this.shooter = Optional.of(shooter);
    dependencyOrderedExecutor.addDependencies(RUN_SHOT_CALCULATOR, ShooterSubsystem.UPDATE_INPUTS);
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
    checkForDuplicateSubsystem(this.hood, "Hood");

    this.hood = Optional.of(hood);

    dependencyOrderedExecutor.registerAction(
        UPDATE_HOOD_DEPENDENCIES, () -> updateHoodDependencies(hood));

    dependencyOrderedExecutor.addDependencies(RUN_SHOT_CALCULATOR, HoodSubsystem.UPDATE_INPUTS);
  }

  public void setHomingSwitch(HomingSwitch homingSwitch) {
    checkForDuplicateSubsystem(this.homingSwitch, "HomingSwitch");

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
   * Coordinates subsystem actions based on the desired action and subsystem inputs
   */
  public void coordinateSubsystemActions() {
  }

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
    Pose2d robotPose = driveInstance.getPose();
    Translation3d shooterPosition =
        new Pose3d(robotPose).plus(JsonConstants.robotInfo.robotToShooter).getTranslation();

    // Pick either passing target or hub here.
    ShotTarget target = ShotTarget.Hub;

    ChassisSpeeds fieldCentricSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            driveInstance.getChassisSpeeds(), robotPose.getRotation());

    /*
      Calculate the additional velocity caused by the rotation of the robot
      The shooter is basically a point a certain radius away from the center of the robot.

      In terms of vectors, this is represented by v_rot = omega x r_vec, where r_vec is the radius vector from the center of the robot to the turret.

      Let r_vec = <x, y, z>. The cross product becomes:

                i j k
      omega_vec 0 0 omega
      r_vec     x y z

      = i(0*z - omega * y) - j(0*z - omega * x) + (0*y - 0*x)
      Which, after simplification equals:
      = - i *omega * y + j * omega * x
      = < - omega * y, omega * x >

      However, we can accomplish this math using Translation3d.cross instead:
    */
    double omega = fieldCentricSpeeds.omegaRadiansPerSecond;
    Translation3d omega_vec = new Translation3d(0, 0, omega);

    Translation3d robotToShooterTranslation =
        JsonConstants.robotInfo.robotToShooter.getTranslation();
    Translation3d fieldRelativeRobotToShooter =
        robotToShooterTranslation.rotateBy(new Rotation3d(robotPose.getRotation()));

    Vector<N3> vRot = omega_vec.cross(fieldRelativeRobotToShooter);

    // Shooter velocity is the instantaneous velocity of the shooter at the moment of release.
    // This means that it must include the translation of the drivetrain, plus the circular motion
    // of the shooter as the drivetrain rotates.
    // This explicitly does not model the curved motion caused by the rotation of the drivetrain
    // over time, which would be represented using Twist2d. This is because the ball will not curve
    // in the air in the same way as the shooter curves on the ground.
    Translation2d shooterVelocity =
        new Translation2d(
            fieldCentricSpeeds.vxMetersPerSecond + vRot.get(0),
            fieldCentricSpeeds.vyMetersPerSecond + vRot.get(1));

    Optional<MapBasedShotInfo> maybeShot =
        ShotCalculations.calculateShotFromMap(shooterPosition, shooterVelocity, target);

    maybeShot.ifPresent(
        shot -> {
          turret.ifPresent(
              turret -> {
                turret.targetGoalHeading(new Rotation2d(shot.yawRadians()));
              });

          hood.ifPresent(
              hood -> {
                hood.targetAngleRadians(shot.hoodAngleRadians());
                ;
              });

          shooter.ifPresent(
              shooter -> {
                shooter.setTargetVelocityRPM(shot.shooterRPM());
              });
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
                    idealShot.pitchRadians(),
                    Math.toRadians(90 - JsonConstants.hoodConstants.maxHoodAngle.in(Degrees)),
                    Math.toRadians(90 - JsonConstants.hoodConstants.minHoodAngle.in(Degrees)))),
        turret
            .map(turret -> turret.getFieldCentricTurretHeading().getRadians())
            .orElse(idealShot.yawRadians()),
        idealShot.timeSeconds());
  }
}
