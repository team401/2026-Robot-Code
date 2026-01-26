package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private Optional<Drive> drive = Optional.empty();
  private Optional<TurretSubsystem> turret = Optional.empty();
  private Optional<HoodSubsystem> hood = Optional.empty();
  // The homing switch will likely be either added to one subsystem or made its own subsystem later
  private Optional<Void> homingSwitch = Optional.empty();

  private final DependencyOrderedExecutor dependencyOrderedExecutor;

  // DOE Action Keys
  public static final ActionKey READ_HOMING_SWITCH =
      new ActionKey("CoordinationLayer::readHomingSwitch");
  public static final ActionKey UPDATE_TURRET_DEPENDENCIES =
      new ActionKey("CoordinationLayer::updateTurretDependencies");
  public static final ActionKey UPDATE_HOOD_DEPENDENCIES =
      new ActionKey("CoordinationLayer::updateHoodDependencies");
  public static final ActionKey RUN_SHOT_CALCULATOR =
      new ActionKey("CoordinationLayer::runShotCalculator");

  // State variables (these will be updated by various methods and then their values will be passed
  // to subsystems during the execution of a cycle)
  private boolean isHomingSwitchPressed = false;

  public CoordinationLayer(DependencyOrderedExecutor dependencyOrderedExecutor) {
    this.dependencyOrderedExecutor = dependencyOrderedExecutor;

    dependencyOrderedExecutor.registerAction(READ_HOMING_SWITCH, this::readHomingSwitch);
    dependencyOrderedExecutor.registerAction(
        UPDATE_TURRET_DEPENDENCIES, this::updateTurretDependencies);
    dependencyOrderedExecutor.registerAction(
        UPDATE_HOOD_DEPENDENCIES, this::updateHoodDependencies);
    dependencyOrderedExecutor.registerAction(RUN_SHOT_CALCULATOR, this::runShotCalculator);
  }

  public void setDrive(Drive drive) {
    if (this.drive.isPresent()) {
      throw new IllegalStateException("CoordinationLayer setDrive was called twice!");
    }

    this.drive = Optional.of(drive);
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
    if (this.turret.isPresent()) {
      throw new IllegalStateException("CoordinationLayer setTurret was called twice!");
    }

    this.turret = Optional.of(turret);
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
    dependencyOrderedExecutor.addDependencies(RUN_SHOT_CALCULATOR, HoodSubsystem.UPDATE_INPUTS);
  }

  // Subsystem dependency updates
  /** Read homing switch state to determine if it's pressed */
  private void readHomingSwitch() {
    // TODO: Add actual check for whether homing switch is pressed once it is designed
    isHomingSwitchPressed = homingSwitch.map(s -> false).orElse(false);
  }

  /** Update the turret subsystem on the state of the homing switch. */
  private void updateTurretDependencies() {
    turret.ifPresent(
        turret -> {
          turret.setIsHomingSwitchPressed(isHomingSwitchPressed);
          drive.ifPresent(
              drive -> {
                turret.setRobotHeading(drive.getRotation());
              });
        });
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
    Translation3d hubTranslation =
        new Translation3d(
            Units.inchesToMeters(182.11),
            Units.inchesToMeters(158.84),
            Units.inchesToMeters(72 - 8));
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
        ShooterCalculations.calculateMovingShot(
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
