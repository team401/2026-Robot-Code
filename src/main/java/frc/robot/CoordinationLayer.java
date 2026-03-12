package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
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
import frc.robot.subsystems.HomingSwitch;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transferroller.TransferRollerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.AllianceUtil;
import frc.robot.util.OptionalUtil;
import frc.robot.util.StateMachineDump;
import frc.robot.util.TestModeManager;
import frc.robot.util.geometry.EnhancedLine2d;
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

  public enum ExtensionState {
    None,
    IntakeDeployed,
    ClimbDeployed
  }

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
   * Tracks our target "extension state": either the intake, climber, or neither may deploy at once.
   */
  @AutoLogOutput(key = "CoordinationLayer/goalExtensionState")
  private ExtensionState goalExtensionState = ExtensionState.None;

  /** Handles making sure that only one subsystem is extended at a time */
  private final StateMachine<CoordinationLayer> extensionStateMachine;

  private final State<CoordinationLayer> noExtensionState;
  private final State<CoordinationLayer> intakeDeployedState;
  private final State<CoordinationLayer> waitForIntakeRetractState;
  private final State<CoordinationLayer> climberDeployedState;
  private final State<CoordinationLayer> waitForClimbRetractState;

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

    this.extensionStateMachine = new StateMachine<CoordinationLayer>(this);

    this.noExtensionState =
        extensionStateMachine.registerState(
            new State<CoordinationLayer>("NoExtension") {
              @Override
              protected void periodic(
                  StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
                intake.ifPresent(IntakeSubsystem::setTargetPositionStowed);

                climber.ifPresent(ClimberSubsystem::stayStowed);
              }
            });

    this.intakeDeployedState =
        extensionStateMachine.registerState(
            new State<CoordinationLayer>("IntakeDeployed") {
              @Override
              protected void periodic(
                  StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
                intake.ifPresent(IntakeSubsystem::setTargetPositionIntaking);

                climber.ifPresent(ClimberSubsystem::stayStowed);
              }
            });

    this.waitForIntakeRetractState =
        extensionStateMachine.registerState(
            new State<CoordinationLayer>("WaitForIntakeRetract") {
              @Override
              protected void periodic(
                  StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
                intake.ifPresent(IntakeSubsystem::setTargetPositionStowed);

                climber.ifPresent(ClimberSubsystem::stayStowed);

                // Assume the intake is stowed if it is disabled
                if (intake.map(IntakeSubsystem::isStowed).orElse(true)) {
                  finish();
                }
              }
            });

    this.climberDeployedState =
        extensionStateMachine.registerState(
            new State<CoordinationLayer>("ClimberDeployed") {
              @Override
              protected void onEntry(
                  StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
                climber.ifPresent(ClimberSubsystem::search);
              }

              @Override
              protected void periodic(
                  StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
                intake.ifPresent(IntakeSubsystem::setTargetPositionStowed);

                /* We don't need to command the climber in periodic; its actions within this state are commanded by individual button bindings. This not might be the cleanest solution, but it will work for now with manual climbing. When we automate driving to climb, the CoordinationLayer state machine will likely need to morph from an extension state machine to a whole robot coordination state machine that tracks driving to climb, extending climber, climbing, and eventually unclimbing in addition to protecting against double extension. */
              }
            });

    this.waitForClimbRetractState =
        extensionStateMachine.registerState(
            new State<CoordinationLayer>("WaitForClimberRetract") {
              @Override
              protected void periodic(
                  StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
                intake.ifPresent(IntakeSubsystem::setTargetPositionStowed);

                climber.ifPresent(ClimberSubsystem::stayStowed);

                // Assume the climber is stowed if it is disabled
                if (climber.map(ClimberSubsystem::isStowedOrHasntBeenHomed).orElse(true)) {
                  finish();
                }
              }
            });

    this.noExtensionState
        .when(() -> goalExtensionState == ExtensionState.IntakeDeployed, "Goal is IntakeDeployed")
        .transitionTo(intakeDeployedState);
    this.noExtensionState
        .when(() -> goalExtensionState == ExtensionState.ClimbDeployed, "Goal is ClimbDeployed")
        .transitionTo(climberDeployedState);

    this.intakeDeployedState
        .when(
            () -> goalExtensionState != ExtensionState.IntakeDeployed, "Goal is not IntakeDeployed")
        .transitionTo(waitForIntakeRetractState);

    this.waitForIntakeRetractState
        .whenFinished("Intake finished retracting")
        .transitionTo(noExtensionState);

    this.waitForIntakeRetractState
        .when(() -> goalExtensionState == ExtensionState.IntakeDeployed, "Goal is IntakeDeployed")
        .transitionTo(intakeDeployedState);

    this.climberDeployedState
        .when(() -> goalExtensionState != ExtensionState.ClimbDeployed, "Goal is not ClimbDeployed")
        .transitionTo(waitForClimbRetractState);

    this.waitForClimbRetractState
        .whenFinished("Climb finished retracting")
        .transitionTo(noExtensionState);

    this.waitForClimbRetractState
        .when(() -> goalExtensionState == ExtensionState.ClimbDeployed, "Goal is ClimbDeployed")
        .transitionTo(climberDeployedState);

    extensionStateMachine.setState(noExtensionState);
    StateMachineDump.write("coordination", extensionStateMachine);

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

    Trigger climbLeft = makeTriggerFromButton(controllers.getButton("climbLeft"));
    Trigger climbRight = makeTriggerFromButton(controllers.getButton("climbRight"));

    climbLeft
        .and(new Trigger(() -> autonomyLevel == AutonomyLevel.Smart))
        .whileTrue(JsonConstants.autos.getRoutineCommandReference("LeftClimbLineup"));
    climbRight
        .and(new Trigger(() -> autonomyLevel == AutonomyLevel.Smart))
        .whileTrue(JsonConstants.autos.getRoutineCommandReference("RightClimbLineup"));

    Trigger eitherClimbPressed = climbLeft.or(climbRight);
    eitherClimbPressed
        .and(new Trigger(() -> autonomyLevel == AutonomyLevel.Manual))
        .onTrue(
            new InstantCommand(
                () -> {
                  // Deploy the climber to a searching state when ready
                  goalExtensionState = ExtensionState.ClimbDeployed;
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  goalExtensionState = ExtensionState.ClimbDeployed;
                  climber.ifPresent(ClimberSubsystem::hang);
                }));

    var goUnderTrenchButton = controllers.getButton("goUnderTrench");
    makeTriggerFromButton(goUnderTrenchButton).onTrue(new InstantCommand(this::goUnderTrench));
    this.isDriverGoUnderTrenchPressed = goUnderTrenchButton.getPrimitiveIsPressedSupplier();

    makeTriggerFromButton(controllers.getButton("stowClimber"))
        .onTrue(new InstantCommand(this::stowClimber));

    makeTriggerFromButton(controllers.getButton("disableAutonomy"))
        .onTrue(new InstantCommand(this::disableAutonomy));

    makeTriggerFromButton(controllers.getButton("enableAutonomy"))
        .onTrue(new InstantCommand(this::enableAutonomy));

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
  /**
   * Deploys the intake mechanism and activates the intake rollers for autonomous operation. Sets
   * the goal extension state to {@link ExtensionState#IntakeDeployed} and enables the intake
   * rollers to begin collecting game pieces.
   */
  public void deployIntakeForAuto() {
    goalExtensionState = ExtensionState.IntakeDeployed;
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
   *   <li>Sets the goal extension state to {@link ExtensionState#None}, retracting the intake.
   *   <li>Stops the intake rollers by setting {@code runningIntakeRollers} to {@code false}.
   * </ul>
   */
  public void stowIntakeForAuto() {
    goalExtensionState = ExtensionState.None;
    runningIntakeRollers = false;
  }

  public void climbSearchForAuto() {
    goalExtensionState = ExtensionState.ClimbDeployed;
    climber.ifPresent(ClimberSubsystem::search);
  }

  public boolean isClimbSearchFinishedForAuto() {
    return climber.map(ClimberSubsystem::isAtSearchPosition).orElse(true);
  }

  public void climbHangForAuto() {
    goalExtensionState = ExtensionState.ClimbDeployed;
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
   * <p>When deploying the intake, intake rollers are automatically enabled.
   *
   * <p>This means that this button can be pressed regardless of current extension state and it will
   * still toggle the intake correctly.
   */
  private void toggleIntakeDeploy() {
    switch (goalExtensionState) {
      case None, ClimbDeployed -> {
        goalExtensionState = ExtensionState.IntakeDeployed;
        runningIntakeRollers = true;
      }
      case IntakeDeployed -> {
        goalExtensionState = ExtensionState.None;
      }
    }
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

  private void stowClimber() {
    if (goalExtensionState == ExtensionState.ClimbDeployed) {
      boolean willClimberStow = climber.map(ClimberSubsystem::stowPressed).orElse(true);

      if (willClimberStow) {
        goalExtensionState = ExtensionState.None;
      }
    }
  }

  private void disableAutonomy() {
    autonomyLevel = AutonomyLevel.Manual;
    stopShooting();
  }

  private void enableAutonomy() {
    autonomyLevel = AutonomyLevel.Smart;
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
  }

  /** Update the hood subsystem on the state of the homing switch. */
  private void updateHoodDependencies(HoodSubsystem hood) {
    hood.setIsHomingSwitchPressed(isHomingSwitchPressed());
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
          intake -> intake.runRollers(JsonConstants.intakeConstants.intakeRollerSpeed));
    } else {
      intake.ifPresent(IntakeSubsystem::stopRollers);
    }

    // Handle extension coordination and command intake/climber to their correct positions
    Logger.recordOutput(
        "CoordinationLayer/extensionState", extensionStateMachine.getCurrentState().getName());
    extensionStateMachine.periodic();
    Logger.recordOutput(
        "CoordinationLayer/extensionStateAfter", extensionStateMachine.getCurrentState().getName());

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

    boolean canShoot =
        isForceShootPressed.getAsBoolean()
            || (shootingEnabled
                && canShootInCurrentMatchState
                && canPassPastNet
                && shooter.map(shooter -> shooter.isAtGoalVelocity(shotMode)).orElse(false)
                && hood.map(hood -> hood.isAimedCorrectly(shotMode)).orElse(false)
                // When the turret isn't enabled, assume that it's been locked into the correct
                // location for a manual mode shot if we ever have to run "no turret"
                && turret.map(turret -> turret.isAimedCorrectly(shotMode)).orElse(true));
    Logger.recordOutput("CoordinationLayer/canShoot", canShoot);

    if (canShoot) {
      hopper.ifPresent(
          hopper -> hopper.setTargetVelocity(JsonConstants.hopperConstants.indexingVelocity));
      indexer.ifPresent(
          indexer -> indexer.setTargetVelocity(JsonConstants.indexerConstants.indexingVelocity));
      transferRoller.ifPresent(
          transferRoller ->
              transferRoller.setTargetVelocity(
                  JsonConstants.transferRollerConstants.transferRollerSpinningVelocity));
    } else {
      hopper.ifPresent(hopper -> hopper.setTargetVelocity(RPM.zero()));
      indexer.ifPresent(indexer -> indexer.setTargetVelocity(RPM.zero()));
      transferRoller.ifPresent(
          transferRoller -> transferRoller.setTargetVelocity(RadiansPerSecond.zero()));
    }

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
            case Manual -> aimForManualShot();
          };
    }

    Logger.recordOutput("CoordinationLayer/isShotReal", isShotReal);

    boolean shouldStowHoodBasedOnMovement =
        autonomyLevel == AutonomyLevel.Smart
            && OptionalUtil.mapTwo(drive, hood, this::shouldStowHoodBasedOnMovement).orElse(false);
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
    if (!shootingEnabled) {
      shooter.ifPresent(shooter -> shooter.stopShooter());
    }
  }

  /**
   * Aim for either passing or scoring in manual mode
   *
   * @return {@code true} to indicate that this shot is "real"
   */
  private boolean aimForManualShot() {
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

        Rotation2d turretHeading =
            AllianceUtil.isRed()
                ? JsonConstants.manualModeConstants.redHubHeading
                : JsonConstants.manualModeConstants.blueHubHeading;
        turret.ifPresent(turret -> turret.targetGoalHeading(turretHeading));
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
        turret.ifPresent(turret -> turret.targetGoalHeading(turretHeading));
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

  private boolean shouldStowHoodBasedOnMovement(Drive drive, HoodSubsystem hood) {
    Pose2d robotPose = drive.getPose();

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
}
