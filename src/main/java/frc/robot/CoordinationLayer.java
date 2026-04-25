package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.vision.VisionLocalizer;
import coppercore.wpilib_interface.controllers.Controller.Button;
import coppercore.wpilib_interface.controllers.Controllers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.ShotCalculations.MapBasedShotInfo;
import frc.robot.ShotCalculations.ShotInfo;
import frc.robot.ShotCalculations.ShotTarget;
import frc.robot.constants.AllianceBasedFieldConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldLocations;
import frc.robot.constants.JsonConstants;
import frc.robot.coordination.CoordinationTestMode;
import frc.robot.coordination.MatchState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.homingswitch.HomingSwitch;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transferroller.TransferRollerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.AllianceUtil;
import frc.robot.util.OptionalUtil;
import frc.robot.util.TestModeManager;
import frc.robot.util.geometry.EnhancedLine2d;
import frc.robot.util.geometry.Rectangle;
import frc.robot.util.math.Lazy;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * The coordination layer is responsible for updating subsystem dependencies, running the shot
 * calculator, and distributing commands to subsystems based on the action layer.
 *
 * <p>It is also responsible for tracking the current robot action based on user inputs in teleop.
 */
public class CoordinationLayer {
  // Subsystems
  private Optional<ClimberSubsystem> climber = Optional.empty();
  private Optional<Drive> drive = Optional.empty();
  private Optional<DriveCoordinator> driveCoordinator = Optional.empty();
  private Optional<VisionLocalizer> vision = Optional.empty();
  private Optional<HopperSubsystem> hopper = Optional.empty();
  private Optional<IndexerSubsystem> indexer = Optional.empty();
  private Optional<TransferRollerSubsystem> transferRoller = Optional.empty();
  private Optional<TurretSubsystem> turret = Optional.empty();
  private Optional<ShooterSubsystem> shooter = Optional.empty();
  private Optional<HoodSubsystem> hood = Optional.empty();
  private Optional<IntakeSubsystem> intake = Optional.empty();
  // The homing switch will likely be either added to one subsystem or made its own subsystem later
  private Optional<HomingSwitch> homingSwitch = Optional.empty();

  private final DependencyOrderedExecutor dependencyOrderedExecutor;

  // DOE Action Keys
  public static final ActionKey UPDATE_TURRET_DEPENDENCIES =
      new ActionKey("CoordinationLayer::updateTurretDependencies");
  public static final ActionKey UPDATE_HOOD_DEPENDENCIES =
      new ActionKey("CoordinationLayer::updateHoodDependencies");
  public static final ActionKey UPDATE_INTAKE_DEPENDENCIES =
      new ActionKey("CoordinationLayer::updateIntakeDependencies");
  public static final ActionKey COORDINATE_ROBOT_ACTIONS =
      new ActionKey("CoordinationLayer::coordinateRobotActions");

  // State variables (these will be updated by various methods and then their values will be passed
  // to subsystems during the execution of a cycle)
  /**
   * This is the EventLoop that should be used for all buttons this season. This is necessary
   * because the CommandScheduler button loop won't run before
   * CoordinationLayer:coordinateRobotActions.
   */
  private final EventLoop buttonLoop = new EventLoop();

  public enum AutonomyLevel {
    Smart,
    Manual
  }

  private final Debouncer visionConnectedDebouncer =
      new Debouncer(
          JsonConstants.visionConstants.disconnectedDebounceTime.in(Seconds),
          DebounceType.kFalling);

  public enum ShotMode {
    Pass,
    Hub
  }

  /**
   * Tracks our current "autonomy level": either vision is enabled & used (smart), or manual driver
   * alignment is used (manual)
   */
  @AutoLogOutput(key = "CoordinationLayer/autonomyLevel")
  private AutonomyLevel autonomyLevel = AutonomyLevel.Smart;

  /**
   * Tracks the "effective autonomy level": if autonomy level is set to smart but we lose vision,
   * our autonomy will automatically be disabled and our effective autonomy level will become
   * manual. These are separate values because an alert should be displayed if effective autonomy
   * level is overridden.
   */
  private AutonomyLevel effectiveAutonomyLevel = autonomyLevel;

  /**
   * Whether the intake should currently be commanded to be deployed.
   *
   * <p>This is called {@code deployIntake} and not {@code intakeDeployed} or {@code
   * isIntakeDeployed} intentionally to denote the difference between whether we are commanding the
   * intake to deploy (an ideal state/command) with whether the intake is deployed (a code input,
   * read from the encoder).
   */
  @AutoLogOutput(key = "CoordinationLayer/deployIntake")
  private boolean deployIntake = false;

  @AutoLogOutput(key = "CoordinationLayer/runningIntakeRollers")
  private boolean runningIntakeRollers = false;

  @AutoLogOutput(key = "CoordinationLayer/shotMode")
  private ShotMode shotMode = ShotMode.Hub;

  /**
   * Whether the shot we were aiming for last cycle was a real shot or an approximation of the
   * nearest possible shot
   *
   * <p>This is needed because, when it encounters a shot that's outside of its shot map, the shot
   * calculator simply clamps the distance to the nearest possible distance and aims for that
   * instead. This is done so that, whenever the circumstances change so that we can shoot again,
   * the robot is already aimed to shoot.
   */
  private boolean isShotReal = false;

  /** Whether or not we should currently be running boosted intake speed in teleop */
  private boolean isIntakeBoosted = false;

  /**
   * Whether or not we are currently in defense mode.
   *
   * <p>Being in defense mode stows the hood, brakes the turret, and sets the drive current limits
   * to a higher value.
   */
  @AutoLogOutput(key = "CoordinationLayer/inDefenseMode")
  private boolean inDefenseMode = false;

  // Tunable numbers for shot tuning
  private final Lazy<LoggedTunableNumber> hoodTuningAngleDegrees =
      new Lazy<>(
          () ->
              new LoggedTunableNumber(
                  "CoordinationLayer/ShotTuning/hoodAngleDegrees",
                  JsonConstants.hoodConstants.minHoodAngle.in(Degrees)));
  private final Lazy<LoggedTunableNumber> shooterTuningRPM =
      new Lazy<>(() -> new LoggedTunableNumber("CoordinationLayer/ShotTuning/shooterRPM", 0.0));
  private final TestModeManager<CoordinationTestMode> testModeManager =
      new TestModeManager<>("CoordinationLayer", CoordinationTestMode.class);

  // Logging
  private final Alert autonomyOverriddenAlert =
      new Alert(
          "Autonomy level forced to manual due to disconnected coprocessor.", AlertType.kWarning);
  private final Alert visionDisconnectedAlert =
      new Alert("Coprocessor disconnected", AlertType.kError);

  // Suppliers from controllers
  private BooleanSupplier isDriverGoUnderTrenchPressed = () -> false;
  private BooleanSupplier isOperatorStowHoodForTrenchPressed = () -> false;
  private BooleanSupplier isWonAutoPressed = () -> false;
  private BooleanSupplier isLostAutoPressed = () -> false;
  private BooleanSupplier isForceShootPressed = () -> false;

  /**
   * Whether or not the robot should currently be shooting.
   *
   * <p>In smart mode, this means the robot will shoot when ready. This means waiting for an
   * achievable shot and waiting for mechanisms to achieve that shot.
   *
   * <p>In manual mode, this means the robot shoot as soon as its mechanisms are in the right
   * positions for manual shooting from the tower.
   */
  @AutoLogOutput(key = "CoordinationLayer/shootingEnabled")
  private boolean shootingEnabled = false;

  private final MatchState matchState = new MatchState();

  public CoordinationLayer(DependencyOrderedExecutor dependencyOrderedExecutor) {
    this.dependencyOrderedExecutor = dependencyOrderedExecutor;

    dependencyOrderedExecutor.registerAction(
        COORDINATE_ROBOT_ACTIONS, this::coordinateRobotActions);

    AutoLogOutputManager.addObject(this);

    initializePositionBasedStrategyTriggers();
  }

  /** Initialize triggers that change the robot's goal scoring location based on position */
  private void initializePositionBasedStrategyTriggers() {
    new Trigger(
            buttonLoop,
            () ->
                drive
                    .map(drive -> AllianceBasedFieldConstants.isInAllianceZone(drive.getPose()))
                    .orElse(false))
        .onTrue(
            new InstantCommand(
                () -> {
                  if (DriverStation.isTeleopEnabled()
                      && effectiveAutonomyLevel == AutonomyLevel.Smart) {
                    shotMode = ShotMode.Hub;
                  }
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (DriverStation.isTeleopEnabled()
                      && effectiveAutonomyLevel == AutonomyLevel.Smart) {
                    shotMode = ShotMode.Pass;
                  }
                }));
  }

  // Controller bindings
  /** Initialize all bindings based on the controllers loaded from JSON */
  public void initBindings() {
    Controllers controllers = JsonConstants.controllers;

    makeTriggerFromButton(controllers.getButton("toggleIntakeDeploy"))
        .onTrue(new InstantCommand(this::toggleIntakeDeploy));
    makeTriggerFromButton(controllers.getButton("toggleIntakeRollers"))
        .onTrue(new InstantCommand(this::toggleIntakeRollers));

    makeTriggerFromButton(controllers.getButton("enterPassMode"))
        .onTrue(new InstantCommand(this::enterPassMode));

    makeTriggerFromButton(controllers.getButton("enterHubMode"))
        .onTrue(new InstantCommand(this::enterHubMode));

    makeTriggerFromButton(controllers.getButton("startShooting"))
        .onTrue(new InstantCommand(this::startShooting));

    makeTriggerFromButton(controllers.getButton("stopShooting"))
        .onTrue(new InstantCommand(this::stopShooting));

    Trigger eitherSlowdownPressed =
        makeTriggerFromButton(controllers.getButton("slowdown1"))
            .or(makeTriggerFromButton(controllers.getButton("slowdown2")));
    eitherSlowdownPressed
        .onTrue(
            new InstantCommand(
                () -> driveCoordinator.ifPresent(DriveCoordinator::slowdownDriveWithJoysticks)))
        .onFalse(
            new InstantCommand(
                () ->
                    driveCoordinator.ifPresent(
                        DriveCoordinator::setDriveWithJoysticksToFullSpeed)));

    var goUnderTrenchButton = controllers.getButton("goUnderTrench");
    makeTriggerFromButton(goUnderTrenchButton).onTrue(new InstantCommand(this::goUnderTrench));
    this.isDriverGoUnderTrenchPressed = goUnderTrenchButton.getPrimitiveIsPressedSupplier();

    makeTriggerFromButton(controllers.getButton("stowClimber"))
        .onTrue(new InstantCommand(this::onStowPressed));

    makeTriggerFromButton(controllers.getButton("disableAutonomy"))
        .onTrue(new InstantCommand(this::disableAutonomy));

    makeTriggerFromButton(controllers.getButton("enableAutonomy"))
        .onTrue(new InstantCommand(this::enableAutonomy));

    makeTriggerFromButton(controllers.getButton("toggleDefenseMode"))
        .onTrue(new InstantCommand(this::toggleDefenseMode));

    makeTriggerFromButton(controllers.getButton("seedHeadingForward"))
        .onTrue(new InstantCommand(this::seedHeadingForward));

    // Operator controller:
    makeTriggerFromButton(controllers.getButton("operatorToggleIntakeDeploy"))
        .onTrue(new InstantCommand(this::toggleIntakeDeploy));

    this.isOperatorStowHoodForTrenchPressed =
        controllers.getButton("operatorStowHood").getPrimitiveIsPressedSupplier();

    var won1Supplier = controllers.getButton("operatorWonAuto1").getPrimitiveIsPressedSupplier();
    var won2Supplier = controllers.getButton("operatorWonAuto2").getPrimitiveIsPressedSupplier();
    this.isWonAutoPressed = () -> won1Supplier.getAsBoolean() || won2Supplier.getAsBoolean();

    var lost1Supplier = controllers.getButton("operatorLostAuto1").getPrimitiveIsPressedSupplier();
    var lost2Supplier = controllers.getButton("operatorLostAuto2").getPrimitiveIsPressedSupplier();
    this.isLostAutoPressed = () -> lost1Supplier.getAsBoolean() || lost2Supplier.getAsBoolean();

    makeTriggerFromButton(controllers.getButton("operatorStartShooting"))
        .onTrue(new InstantCommand(this::startShooting));

    this.isForceShootPressed =
        controllers.getButton("operatorForceShoot").getPrimitiveIsPressedSupplier();

    makeTriggerFromButton(controllers.getButton("operatorStopShooting"))
        .onTrue(new InstantCommand(this::stopShooting));

    makeTriggerFromButton(controllers.getButton("operatorEnterPassMode"))
        .onTrue(new InstantCommand(this::enterPassMode));

    makeTriggerFromButton(controllers.getButton("operatorEnterHubMode"))
        .onTrue(new InstantCommand(this::enterHubMode));

    makeTriggerFromButton(controllers.getButton("operatorDisableAutonomy"))
        .onTrue(new InstantCommand(this::disableAutonomy));

    makeTriggerFromButton(controllers.getButton("operatorEnableAutonomy"))
        .onTrue(new InstantCommand(this::enableAutonomy));

    makeTriggerFromButton(controllers.getButton("operatorIncreaseRPM"))
        .onTrue(new InstantCommand(this::increaseRPM));

    makeTriggerFromButton(controllers.getButton("operatorDecreaseRPM"))
        .onTrue(new InstantCommand(this::decreaseRPM));

    makeTriggerFromButton(controllers.getButton("operatorHoldForBoost"))
        .onTrue(new InstantCommand(this::boostIntakeRPM))
        .onFalse(new InstantCommand(this::stopBoostingIntakeRPM));
  }

  /**
   * Create a Trigger from a condition supplier on the CoordinationLayer's custom button loop.
   *
   * @param condition A BooleanSupplier providing the condition (e.g.
   *     Button.getPrimitiveIsPressedSupplier())
   * @return A new Trigger on the custom button loop
   */
  private Trigger makeTriggerFromCondition(BooleanSupplier condition) {
    return new Trigger(buttonLoop, condition);
  }

  /**
   * Create a Trigger from a coppercore wpilib_interface Button.
   *
   * <p>Uses makeTriggerFromCondition on the primitive isPressed supplier provided by the button.
   *
   * @param button The Button from which to make the supplier
   * @return A new Trigger on the CoordinationLayer's custom event loop
   */
  private Trigger makeTriggerFromButton(Button button) {
    return makeTriggerFromCondition(button.getPrimitiveIsPressedSupplier());
  }

  // AUTO METHOD
  // Only autos are allowed to call these methods

  // Docs Written by Claude Opus 4.6
  /** Deploys the intake mechanism and activates the intake rollers for autonomous operation. */
  public void deployIntakeForAuto() {
    deployIntake = true;
    runningIntakeRollers = true;
  }

  // Docs Written by Claude Opus 4.6
  /**
   * Stows the intake mechanism for autonomous mode by retracting the extension and stopping the
   * intake rollers.
   *
   * <p>This method should be called before or during autonomous routines to ensure the intake is in
   * a safe, retracted position and not actively running.
   *
   * <p>Effects:
   *
   * <ul>
   *   <li>Retracts the intake by setting {@code deployIntake} to {@code false}.
   *   <li>Stops the intake rollers by setting {@code runningIntakeRollers} to {@code false}.
   * </ul>
   */
  public void stowIntakeForAuto() {
    deployIntake = false;
    runningIntakeRollers = false;
  }

  public void climbSearchForAuto() {
    climber.ifPresent(ClimberSubsystem::search);
  }

  public boolean isClimbSearchFinishedForAuto() {
    return climber.map(ClimberSubsystem::isAtSearchPosition).orElse(true);
  }

  public void climbHangForAuto() {
    climber.ifPresent(ClimberSubsystem::hang);
  }

  public void startShootingForAuto() {
    shootingEnabled = true;
  }

  public void stopShootingForAuto() {
    shootingEnabled = false;
  }

  /**
   * If the climber is deployed, stow it and then deploy the intake. If nothing is deployed, deploy
   * the intake. If the intake is deployed, stow the intake.
   *
   * <p>When deploying the intake, intake rollers are automatically enabled. When retracting the
   * intake, intake rollers are automatically disabled.
   *
   * <p>This means that this button can be pressed regardless of current extension state and it will
   * still toggle the intake correctly.
   */
  private void toggleIntakeDeploy() {
    deployIntake = !deployIntake;
    runningIntakeRollers = deployIntake;
  }

  /**
   * Toggles the intake rollers. Note that whenever the intake is deployed, the rollers are
   * automatically started.
   */
  private void toggleIntakeRollers() {
    runningIntakeRollers = !runningIntakeRollers;
  }

  private void enterPassMode() {
    shotMode = ShotMode.Pass;
  }

  private void enterHubMode() {
    shotMode = ShotMode.Hub;
  }

  private void startShooting() {
    shootingEnabled = true;
  }

  private void stopShooting() {
    shootingEnabled = false;
  }

  /**
   * When in smart mode, starts autonomously driving under the trench. When in manual mode, does
   * nothing.
   *
   * <p>The method coordinateRobotActions is responsible for stowing the hood when this button is
   * pressed in manual autonomy using the associated BooleanSupplier.
   */
  private void goUnderTrench() {
    if (this.autonomyLevel == AutonomyLevel.Smart) {
      // TODO: Add smart mode go under trench behavior after auto-trench-drive is merged.
    }
  }

  /**
   * Handles a press of the stow button by:
   *
   * <ul>
   *   <li>If the climber is hanging, raise it to search position
   *   <li>If the climber is searching, stow it
   */
  private void onStowPressed() {
    climber.ifPresent(
        climber -> {
          if (climber.isHanging()) {
            climber.search();
          } else {
            climber.stow();
          }
        });
  }

  private void disableAutonomy() {
    autonomyLevel = AutonomyLevel.Manual;
    stopShooting();
  }

  private void enableAutonomy() {
    autonomyLevel = AutonomyLevel.Smart;
  }

  private void toggleDefenseMode() {
    inDefenseMode = !inDefenseMode;

    if (inDefenseMode) {
      raiseDriveSupplyCurrentLimitForDefense();
    } else {
      lowerDriveSupplyCurrentLimit();
    }

    drive.ifPresent(drive -> drive.setInDefenseMode(inDefenseMode));
  }

  private void seedHeadingForward() {
    drive.ifPresent(Drive::seedHeadingForward);
  }

  private void increaseRPM() {
    rpmCompensation.setValue(rpmCompensation.getAsDouble() + 20);
  }

  private void decreaseRPM() {
    rpmCompensation.setValue(rpmCompensation.getAsDouble() - 20);
  }

  private void boostIntakeRPM() {
    isIntakeBoosted = true;
  }

  private void stopBoostingIntakeRPM() {
    isIntakeBoosted = false;
  }

  public void lowerDriveSupplyCurrentLimit() {
    this.drive.ifPresent(
        drive -> {
          drive.setSupplyCurrentLimit(
              JsonConstants.physicalDriveConstants.driveSupplyCurrentTeleopLimit);
        });
  }

  private void raiseDriveSupplyCurrentLimitForDefense() {
    this.drive.ifPresent(
        drive -> {
          drive.setSupplyCurrentLimit(
              JsonConstants.physicalDriveConstants.driveSupplyCurrentDefenseLimit);
        });
  }

  // Subsystem initialization

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

  public void setClimber(ClimberSubsystem climber) {
    checkForDuplicateSubsystem(this.climber, "Climber");
    this.climber = Optional.of(climber);
  }

  public void setDrive(Drive drive) {
    checkForDuplicateSubsystem(this.drive, "Drive");
    this.drive = Optional.of(drive);
  }

  public void setDriveCoordinator(DriveCoordinator driveCoordinator) {
    checkForDuplicateSubsystem(this.driveCoordinator, "DriveCoordinator");
    this.driveCoordinator = Optional.of(driveCoordinator);
  }

  public void setVisionLocalizer(VisionLocalizer vision) {
    checkForDuplicateSubsystem(this.vision, "VisionLocalizer");
    this.vision = Optional.of(vision);
  }

  public void setHopper(HopperSubsystem hopper) {
    checkForDuplicateSubsystem(this.hopper, "Hopper");
    this.hopper = Optional.of(hopper);
  }

  public void setIndexer(IndexerSubsystem indexer) {
    checkForDuplicateSubsystem(this.indexer, "Indexer");
    this.indexer = Optional.of(indexer);
  }

  public void setTransferRoller(TransferRollerSubsystem transferRoller) {
    checkForDuplicateSubsystem(this.transferRoller, "TransferRoller");
    this.transferRoller = Optional.of(transferRoller);
  }

  public void setIntake(IntakeSubsystem intake) {
    checkForDuplicateSubsystem(this.intake, "Intake");
    this.intake = Optional.of(intake);

    dependencyOrderedExecutor.registerAction(
        UPDATE_INTAKE_DEPENDENCIES, () -> updateIntakeDependencies(intake));

    if (JsonConstants.featureFlags.runHood) {
      dependencyOrderedExecutor.addDependencies(
          UPDATE_HOOD_DEPENDENCIES, UPDATE_INTAKE_DEPENDENCIES);
    }

    if (JsonConstants.featureFlags.runTurret) {
      dependencyOrderedExecutor.addDependencies(
          UPDATE_TURRET_DEPENDENCIES, UPDATE_INTAKE_DEPENDENCIES);
    }
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

    dependencyOrderedExecutor.addDependencies(
        COORDINATE_ROBOT_ACTIONS, TurretSubsystem.UPDATE_INPUTS);
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
    dependencyOrderedExecutor.addDependencies(
        COORDINATE_ROBOT_ACTIONS, ShooterSubsystem.UPDATE_INPUTS);
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

    dependencyOrderedExecutor.addDependencies(
        COORDINATE_ROBOT_ACTIONS, HoodSubsystem.UPDATE_INPUTS);
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
    if (JsonConstants.featureFlags.runIntake) {
      dependencyOrderedExecutor.addDependencies(
          UPDATE_INTAKE_DEPENDENCIES, HomingSwitch.UPDATE_INPUTS);
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
    // Piggyback off of the intake stopping logic to save power during defense
    turret.shouldStopMoving(
        inDefenseMode || intake.map(IntakeSubsystem::shouldStopTurret).orElse(false));
  }

  /** Update the hood subsystem on the state of the homing switch. */
  private void updateHoodDependencies(HoodSubsystem hood) {
    hood.setIsHomingSwitchPressed(isHomingSwitchPressed());
    // Piggyback off of the intake stowing logic to save power during defense
    hood.setShouldStowForIntakeOrDefense(
        inDefenseMode || intake.map(IntakeSubsystem::shouldStartStowingHood).orElse(false));
  }

  /** Update the intake subsystem on the state of the homing switch. */
  private void updateIntakeDependencies(IntakeSubsystem intake) {
    intake.setIsHomingSwitchPressed(isHomingSwitchPressed());
  }

  // Coordination and processing
  private final EnhancedLine2d redNet =
      new EnhancedLine2d(
          FieldConstants.Hub.oppNearLeftCorner().plus(new Translation2d(-0.27, 0.13)),
          FieldConstants.Hub.oppNearRightCorner().plus(new Translation2d(-0.27, -0.11)));
  private final EnhancedLine2d blueNet =
      new EnhancedLine2d(
          FieldConstants.Hub.farLeftCorner().plus(new Translation2d(0.27, 0.13)),
          FieldConstants.Hub.farRightCorner().plus(new Translation2d(0.27, -0.11)));

  /**
   * This method is like the "periodic" of the CoordinationLayer. It is run by the
   * DependencyOrderedExecutor and is responsible for polling our custom button loop, reading robot
   * goal actions, deciding the best subsystem actions based on the goal robot actions, and applying
   * commands to subsystems. This method must run after subsystems update inputs but before
   * CommandScheduler.run, as it will tell subsystems which commands to apply when their periodic
   * methods run.
   */
  public void coordinateRobotActions() {
    long startTimeUs = RobotController.getFPGATime();

    if (DriverStation.isTest()) {
      // Log distance to hub for test modes
      drive.ifPresent(
          drive -> {
            Logger.recordOutput(
                "CoordinationLayer/distanceToHub",
                AllianceBasedFieldConstants.hubInnerCenterPoint
                    .get()
                    .toTranslation2d()
                    .getDistance(
                        new Pose3d(drive.getPose())
                            .plus(JsonConstants.robotInfo.robotToShooter)
                            .getTranslation()
                            .toTranslation2d()));
          });
    }

    updateMatchState();

    // Poll buttons. This will modify state variables depending on the states of the buttons
    buttonLoop.poll();

    // Allow running rollers when the intake is retracted
    if (runningIntakeRollers) {
      intake.ifPresent(
          intake -> {
            var rollerSpeed = JsonConstants.intakeConstants.intakeTeleOpRollerSpeed;
            if (isIntakeBoosted) {
              rollerSpeed = JsonConstants.intakeConstants.intakeTeleOpBoostedRollerSpeed;
            }
            if (DriverStation.isAutonomous()) {
              rollerSpeed = JsonConstants.intakeConstants.intakeAutoRollerSpeed;
            }
            intake.runRollers(rollerSpeed);
          });
    } else {
      intake.ifPresent(IntakeSubsystem::stopRollers);
    }

    // Holds the intake down with a set voltage when intaking
    if (runningIntakeRollers && deployIntake) {
      intake.ifPresent(
          intake -> {
            intake.controlPivotMotorIOWithVoltage(
                JsonConstants.intakeConstants.pivotVoltageWhenIntaking);
          });
    }

    // Handle intake deploy/retract
    if (deployIntake) {
      intake.ifPresent(IntakeSubsystem::deploy);
    } else {
      intake.ifPresent(IntakeSubsystem::stow);
    }

    // Determine if vision is enabled and functioning
    boolean visionConnected = vision.map(VisionLocalizer::coprocessorConnected).orElse(false);
    boolean visionConnectedDebounced = visionConnectedDebouncer.calculate(visionConnected);
    Logger.recordOutput("CoordinationLayer/visionConnected", visionConnectedDebounced);

    visionDisconnectedAlert.set(JsonConstants.featureFlags.runVision && !visionConnected);

    effectiveAutonomyLevel = visionConnectedDebounced ? autonomyLevel : AutonomyLevel.Manual;
    Logger.recordOutput("CoordinationLayer/effectiveAutonomyLevel", effectiveAutonomyLevel);

    autonomyOverriddenAlert.set(effectiveAutonomyLevel != autonomyLevel);

    // Test whether we can shoot BEFORE running the shot calculator so that we can shoot for the
    // shot we were looking ahead to last cycle.
    // This should improve the performance of shoot on the move.
    // If this isn't sufficient, we can calculate 2 shots: one with compensation delay and one
    // without compensation delay, and then just test the one without compensation delay.
    boolean canShootInCurrentMatchState = this.shotMode == ShotMode.Pass || matchState.canScore();

    // canShootInCurrentZone is true when either:
    // - we are in our alliance zone (to ensure that the autos do not try to score outside our zone)
    // or:
    // - we are in teleop
    boolean canShootInCurrentZone =
        drive
                .map(drive -> AllianceBasedFieldConstants.isInAllianceZone(drive.getPose()))
                .orElse(true)
            || !DriverStation.isAutonomous();

    // canPassPastNet is true when either:
    // - We are not in passing mode (so net isn't a concern)
    // OR
    // - The line from the robot to its passing target doesn't intercept either net
    boolean canPassPastNet =
        this.shotMode != ShotMode.Pass
            || drive
                .map(
                    drive -> {
                      Pose2d pose = drive.getPose();
                      EnhancedLine2d lineToShot =
                          new EnhancedLine2d(
                              pose.getTranslation(), getShotTargetFromPose(pose).getTranslation());

                      return !(lineToShot.intersects(redNet) || lineToShot.intersects(blueNet));
                    })
                .orElse(true);

    // First, verify if the match state & shooter+hood+turret allow us to shoot right now
    // This means that if we were to run the indexer (uptake), the fuel would likely end up in the
    // right place
    boolean aimedAndCanShoot =
        shootingEnabled
            && (isForceShootPressed.getAsBoolean()
                || (canShootInCurrentMatchState
                    && canPassPastNet
                    && shooter.map(shooter -> shooter.isAtGoalVelocity(shotMode)).orElse(false)
                    && hood.map(hood -> hood.isAimedCorrectly(shotMode)).orElse(false)
                    // When the turret isn't enabled, assume that it's been locked into the correct
                    // location for a manual mode shot if we ever have to run "no turret"
                    && turret.map(turret -> turret.isAimedCorrectly(shotMode)).orElse(true)
                    && canShootInCurrentZone));
    Logger.recordOutput("CoordinationLayer/aimedAndCanShoot", aimedAndCanShoot);

    // Only start up the indexer once the shooter, turret, and hood are ready to go (or if
    // force-shoot is pressed)
    if (aimedAndCanShoot) {
      indexer.ifPresent(
          indexer -> indexer.setTargetVelocity(JsonConstants.indexerConstants.indexingVelocity));
    } else {
      indexer.ifPresent(indexer -> indexer.setTargetVelocity(RPM.zero()));
    }

    // Next, verify if the indexer is ready to shoot as well (or ignore it, if force shoot is
    // pressed)
    boolean indexerReady =
        indexer.map(IndexerSubsystem::readyToShoot).orElse(false)
            || isForceShootPressed.getAsBoolean();

    // This maintains the protection for "only force shoot if the shooter wheels are spinning"
    // because aimedAndCanShoot bakes that value into its and condition.
    // If force shoot is pressed but the shooter is disabled, aimedAndCanShoot will remain false so
    // this condition will also be false.
    if (aimedAndCanShoot && indexerReady) {
      hopper.ifPresent(
          hopper -> hopper.setTargetVelocity(JsonConstants.hopperConstants.indexingVelocity));
      transferRoller.ifPresent(
          transferRoller ->
              transferRoller.setTargetVelocity(
                  JsonConstants.transferRollerConstants.transferRollerSpinningVelocity));
    } else {
      hopper.ifPresent(hopper -> hopper.setTargetVelocity(RPM.zero()));
      transferRoller.ifPresent(
          transferRoller -> transferRoller.setTargetVelocity(RadiansPerSecond.zero()));
    }

    long shotCalculationStartTimeUs = RobotController.getFPGATime();
    // Aim for a shot based on the current autonomy level
    if (testModeManager.isInTestMode()) {
      drive.ifPresent(this::aimForTestModeShot);
      // When in test mode, always assume the shot is real to avoid locking ourselves out of shots.
      // Assume that the tuner knows what they are doing.
      isShotReal = true;
    } else {
      isShotReal =
          switch (effectiveAutonomyLevel) {
            // If the drivetrain doesn't exist, we won't shoot. Not sure when this would ever come
            // into play. If this ever becomes an outreach bot, this will need to change.
            case Smart -> drive.map(this::runShotCalculatorWithDrive).orElse(false);
            case Manual -> drive.map(this::aimForManualShot).orElse(false);
          };
    }
    long shotCalculationEndTimeUs = RobotController.getFPGATime();
    if (JsonConstants.featureFlags.logPeriodicTiming) {
      Logger.recordOutput(
          "PeriodicTime/CoordinateRobotActions/shotCalculationMs",
          (shotCalculationEndTimeUs - shotCalculationStartTimeUs) / 1000.0);
    }

    Logger.recordOutput("CoordinationLayer/isShotReal", isShotReal);

    boolean shouldStowHoodBasedOnMovement =
        OptionalUtil.mapTwo(drive, hood, this::shouldStowHoodBasedOnMovement).orElse(false);
    Logger.recordOutput(
        "CoordinationLayer/shouldStowHoodBasedOnMovement", shouldStowHoodBasedOnMovement);
    boolean shouldStowHoodBasedOnButtons =
        isDriverGoUnderTrenchPressed.getAsBoolean()
            || isOperatorStowHoodForTrenchPressed.getAsBoolean();

    boolean shouldStowHood = shouldStowHoodBasedOnButtons || shouldStowHoodBasedOnMovement;
    // Don't need to log shouldStowHood here as it's logged in the hood subsystem
    hood.ifPresent(hood -> hood.setShouldStowForTrench(shouldStowHood));

    // If shooting isn't enabled, stop the shooter flywheels to avoid wasting
    // a ton of energy
    // Also stop them if we're in defense mode or intake is stowed to save power/prevent tearing the
    // net
    if (!shootingEnabled
        || inDefenseMode
        || intake.map(IntakeSubsystem::shouldStartStowingHood).orElse(false)) {
      shooter.ifPresent(shooter -> shooter.stopShooter());
    }

    long endTimeUs = RobotController.getFPGATime();
    if (JsonConstants.featureFlags.logPeriodicTiming) {
      Logger.recordOutput(
          "PeriodicTime/CoordinateRobotActions/totalMs", (endTimeUs - startTimeUs) / 1000.0);
    }
  }

  /**
   * Aim for either passing or scoring in manual mode
   *
   * @return {@code true} to indicate that this shot is "real"
   */
  private boolean aimForManualShot(Drive drive) {
    switch (shotMode) {
      case Hub -> {
        double distanceMeters = JsonConstants.manualModeConstants.assumedHubDistance.in(Meters);

        double hoodAngleRadians =
            JsonConstants.shotMaps.hubMap.hoodAngleRadiansByDistanceMeters().get(distanceMeters);
        hood.ifPresent(
            hood -> {
              hood.targetAngleRadians(hoodAngleRadians);
            });

        double shooterRPM =
            JsonConstants.shotMaps.passingMap.rpmByDistanceMeters().get(distanceMeters);
        shooter.ifPresent(shooter -> shooter.setTargetVelocityRPM(shooterRPM));

        // Command the turret to follow the drivetrain's current heading.
        // This serves 2 purposes:
        // #1 - Gives the driver the ability to aim in manual mode, regardless of gyro functionality
        // #2 - Stops us from tearing a hole in the net when we stow the intake by allowing the
        // turret to be "disabled" by entering manual mode. Since the turret is held at a fixed
        // angle relative to the robot, it won't move and won't tear the net.
        turret.ifPresent(turret -> turret.targetGoalHeading(drive.getRotation()));
      }
      case Pass -> {
        double distanceMeters = JsonConstants.manualModeConstants.assumedPassDistance.in(Meters);

        double hoodAngleRadians =
            JsonConstants.shotMaps
                .passingMap
                .hoodAngleRadiansByDistanceMeters()
                .get(distanceMeters);
        hood.ifPresent(
            hood -> {
              hood.targetAngleRadians(hoodAngleRadians);
            });

        double shooterRPM =
            JsonConstants.shotMaps.passingMap.rpmByDistanceMeters().get(distanceMeters);
        shooter.ifPresent(shooter -> shooter.setTargetVelocityRPM(shooterRPM));

        Rotation2d turretHeading =
            AllianceUtil.isRed()
                ? JsonConstants.manualModeConstants.redPassHeading
                : JsonConstants.manualModeConstants.bluePassHeading;
        turret.ifPresent(turret -> turret.targetGoalHeading(drive.getRotation()));
      }
    }

    // Manual shot is always real, as the manual shot we're aiming for is a shot that we know is
    // possible.
    return true;
  }

  private void aimForTestModeShot(Drive driveInstance) {
    double hoodAngleRadians = Units.degreesToRadians(hoodTuningAngleDegrees.get().getAsDouble());
    double shooterRPM = shooterTuningRPM.get().getAsDouble();

    hood.ifPresent(
        hood -> {
          hood.targetAngleRadians(hoodAngleRadians);
        });

    shooter.ifPresent(shooter -> shooter.setTargetVelocityRPM(shooterRPM));

    Pose2d robotPose = driveInstance.getPose();
    Translation2d shooterPosition =
        robotPose.plus(JsonConstants.robotInfo.robotToShooter2d).getTranslation();

    ShotTarget target = getShotTargetFromPose(robotPose);

    Translation2d targetPose =
        switch (target) {
          case Hub -> AllianceBasedFieldConstants.hubCenterPoint2d.get();
          case PassLeft -> FieldLocations.leftPassingTarget();
          case PassRight -> FieldLocations.rightPassingTarget();
        };

    Rotation2d yaw = targetPose.minus(shooterPosition).getAngle();

    turret.ifPresent(turret -> turret.targetGoalHeading(yaw));
  }

  private final EnhancedLine2d leftBlueTrench =
      new EnhancedLine2d(
          FieldConstants.LeftTrench.openingTopLeft(), FieldConstants.LeftTrench.openingTopRight());
  private final EnhancedLine2d rightRedTrench =
      new EnhancedLine2d(
          FieldConstants.LeftTrench.oppOpeningTopLeft(),
          FieldConstants.LeftTrench.oppOpeningTopRight());
  private final EnhancedLine2d rightBlueTrench =
      new EnhancedLine2d(
          FieldConstants.RightTrench.openingTopLeft(),
          FieldConstants.RightTrench.openingTopRight());
  private final EnhancedLine2d leftRedTrench =
      new EnhancedLine2d(
          FieldConstants.RightTrench.oppOpeningTopLeft(),
          FieldConstants.RightTrench.oppOpeningTopRight());
  private final EnhancedLine2d[] trenches = {
    leftBlueTrench, rightRedTrench, rightBlueTrench, leftRedTrench
  };

  // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf pg
  // 5-6, 8
  private final double SAFETY_WIDTH =
      44.4 * 0.0254 * 2.5; // Andymark bump: length of the side parallel to field's x-axis
  // Bump width is not specified in Welded drawing, assumed to be identical
  private final double SAFETY_HEIGHT =
      // 49.86 * 0.0254; // Andymark width of trench; this is a height on the y-axis of the field
      50.34 * 0.0254; // Welded width of trench; this is a height on the y-axis of the field
  // coordinate system
  private final Rectangle[] trenchZones =
      new Rectangle[] {
        Rectangle.fromCenter(leftBlueTrench.midPoint(), SAFETY_WIDTH, SAFETY_HEIGHT),
        Rectangle.fromCenter(rightBlueTrench.midPoint(), SAFETY_WIDTH, SAFETY_HEIGHT),
        Rectangle.fromCenter(leftRedTrench.midPoint(), SAFETY_WIDTH, SAFETY_HEIGHT),
        Rectangle.fromCenter(rightRedTrench.midPoint(), SAFETY_WIDTH, SAFETY_HEIGHT)
      };

  private boolean shouldStowHoodBasedOnMovement(Drive drive, HoodSubsystem hood) {
    Pose2d robotPose = drive.getPose();

    // we use two methods to protect the hood.
    // first, we check if the robot is currently within a protected rectangle
    // around each trench, regardless of its speed. If so, we stow the hood
    for (var protectedZone : trenchZones) {
      if (protectedZone.contains(robotPose.getTranslation())) {
        return true;
      }
    }

    // second, we project the robot's movement out by timeToStowHood seconds
    // and check if that intersects with one of the trench lines
    ChassisSpeeds robotRelativeSpeeds = drive.getChassisSpeeds();
    Translation2d fieldCentricSpeeds =
        new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d predictedMovement =
        fieldCentricSpeeds.times(JsonConstants.hoodConstants.timeToStowHood.in(Seconds));
    Translation2d shooterPose =
        new Pose3d(robotPose)
            .plus(JsonConstants.robotInfo.robotToShooter)
            .getTranslation()
            .toTranslation2d();

    Translation2d movementStart = shooterPose;
    Translation2d movementEnd = movementStart.plus(predictedMovement);

    for (var protectedZone : trenchZones) {
      if (protectedZone.contains(movementEnd)) {
        return true;
      }
    }

    Logger.recordOutput(
        "CoordinationLayer/ShooterTrajectory", new Translation2d[] {movementStart, movementEnd});
    Logger.recordOutput(
        "CoordinationLayer/HoodPredictedLocation",
        new Pose2d(movementEnd, robotPose.getRotation()));
    EnhancedLine2d movementLine = new EnhancedLine2d(movementStart, movementEnd);

    for (var trench : trenches) {
      if (movementLine.intersects(trench)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Given a robot pose, check whether or not that pose is on the left half of the field.
   *
   * @param robotPose The robot pose from vision and odometry to test.
   * @return {@code true} if on the left side of the field, {@code false} otherwise.
   */
  private boolean isOnLeftSideOfField(Pose2d robotPose) {
    boolean isRed = AllianceUtil.isRed();

    if (isRed) {
      // Blue origin: if we're on red, the left side is -y from the center line
      return robotPose.getY() <= FieldConstants.fieldWidth() / 2;
    } else {
      // If we're on blue, the left side is +y from the center line.
      return robotPose.getY() > FieldConstants.fieldWidth() / 2;
    }
  }

  /**
   * Gets the current ShotTarget based on the current value of shotMode
   *
   * <p>This means either returning ShotTarget.Hub if in hub mode, or finding the correct
   * (left/right) pass target when in passing mode.
   *
   * @param robotPose A Pose2d containing the current position of the robot.
   * @return A ShotTarget containing the correct target to shoot at based on shotMode and robot
   *     position.
   */
  private ShotTarget getShotTargetFromPose(Pose2d robotPose) {
    return switch (this.shotMode) {
      case Hub -> ShotTarget.Hub;
      case Pass -> isOnLeftSideOfField(robotPose) ? ShotTarget.PassLeft : ShotTarget.PassRight;
    };
  }

  private final LoggedTunableNumber rpmCompensation =
      new LoggedTunableNumber(
          "CoordinationLayer/compensationRPM", JsonConstants.shotMaps.rpmCompensation.in(RPM));

  /**
   * Run the shot calculations, given an actual drive instance
   *
   * <p>This method exists to reduce the indentation in runShotCalculator introduced by widespread
   * use of optionals
   *
   * @param drive A Drive instance
   * @return whether or not the shot currently being aimed for is "attainable": {@code true} if
   *     there is a possible shot and the robot is aiming for it, {@code false} if the calculator
   *     couldn't find a possible shot and so is aiming for the "closest thing" to a possible shot.
   */
  private boolean runShotCalculatorWithDrive(Drive driveInstance) {
    Pose2d robotPose = driveInstance.getPose();
    Pose2d shooterPose = robotPose.plus(JsonConstants.robotInfo.robotToShooter2d);

    Logger.recordOutput("CoordinationLayer/shooterPose", shooterPose);

    ShotTarget target = getShotTargetFromPose(robotPose);

    ChassisSpeeds robotRelativeSpeeds = driveInstance.getChassisSpeeds();
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
    double omega = robotRelativeSpeeds.omegaRadiansPerSecond;
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

    MapBasedShotInfo shot =
        ShotCalculations.calculateShotFromMap(
            robotPose, robotRelativeSpeeds, shooterVelocity, target);

    turret.ifPresent(
        turret -> {
          turret.targetGoalHeading(shot.yaw());
        });

    hood.ifPresent(
        hood -> {
          hood.targetAngleRadians(shot.hoodAngleRadians());
        });

    shooter.ifPresent(
        shooter -> {
          shooter.setTargetVelocityRPM(shot.shooterRPM() + rpmCompensation.getAsDouble());
        });

    return shot.isReal();
  }

  /** Update the MatchState each periodic loop */
  private void updateMatchState() {
    if (DriverStation.isEnabled()) {
      matchState.enabledPeriodic(isWonAutoPressed.getAsBoolean(), isLostAutoPressed.getAsBoolean());
    } else {
      matchState.disabledPeriodic();
    }

    // This is temporary code left here to make it easy to integrate the time left functionality
    // with LEDs and superstructure coordination later.
    double timeLeft = matchState.getTimeLeftInCurrentShift();

    boolean hasFiveSecondsLeft = timeLeft >= 5.0;
    Logger.recordOutput("MatchState/has5sLeft", hasFiveSecondsLeft);
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
          hood.targetExitPitch(Radians.of(idealShot.pitchRadians()));
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
        hood.map(hood -> hood.getCurrentExitPitch().in(Radians))
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

  public Optional<TurretSubsystem> getTurret() {
    return turret;
  }

  public Optional<HoodSubsystem> getHood() {
    return hood;
  }

  public Optional<IntakeSubsystem> getIntake() {
    return intake;
  }

  public Optional<ClimberSubsystem> getClimber() {
    return climber;
  }

  public Optional<Drive> getDrive() {
    return drive;
  }
}
