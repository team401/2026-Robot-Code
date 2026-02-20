package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import coppercore.wpilib_interface.controllers.Controller.Button;
import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.controllers.Controllers;
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
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.ShotCalculations.MapBasedShotInfo;
import frc.robot.ShotCalculations.ShotInfo;
import frc.robot.ShotCalculations.ShotTarget;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.HomingSwitch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.ejml.equation.Operation.Extents;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The coordination layer is responsible for updating subsystem dependencies, running the shot
 * calculator, and distributing commands to subsystems based on the action layer.
 *
 * <p>It is also responsible for tracking the current robot action based on user inputs in teleop.
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
   * Tracks our target "extension state": either the intake, climber, or neither may deploy at once.
   */
  @AutoLogOutput(key = "CoordinationLayer/goalExtensionState")
  private ExtensionState goalExtensionState = ExtensionState.None;

  /**
   * Handles making sure that only one subsystem is extended at a time
   */
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

  public CoordinationLayer(DependencyOrderedExecutor dependencyOrderedExecutor) {
    this.dependencyOrderedExecutor = dependencyOrderedExecutor;

    dependencyOrderedExecutor.registerAction(
        COORDINATE_ROBOT_ACTIONS, this::coordinateRobotActions);
    
    this.extensionStateMachine = new StateMachine<CoordinationLayer>(this);

    this.noExtensionState = extensionStateMachine.registerState(new State<CoordinationLayer>("NoExtension") {
      @Override
      protected void periodic(StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
        intake.ifPresent(IntakeSubsystem::setTargetPositionStowed);
        // TODO: Add climber stow method call when it is defined
      }
    });

    this.intakeDeployedState = extensionStateMachine.registerState(new State<CoordinationLayer>("IntakeDeployed") {
      @Override
      protected void periodic(StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
        intake.ifPresent(IntakeSubsystem::setTargetPositionIntaking);
        // TODO: Add climber stow method call when it is defined
      }
    });

    this.waitForIntakeRetractState = extensionStateMachine.registerState(new State<CoordinationLayer>("WaitForIntakeRetract") {
      @Override
      protected void periodic(StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
        intake.ifPresent(IntakeSubsystem::setTargetPositionStowed);
        // TODO: Add climber stow method call when it is defined

        // Assume the intake is stowed if it is disabled
        if (intake.map(IntakeSubsystem::isStowed).orElse(true)) {
          finish();
        }
      }
    });

    this.climberDeployedState = extensionStateMachine.registerState(new State<CoordinationLayer>("ClimberDeployed") {
      @Override
      protected void periodic(StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
        intake.ifPresent(IntakeSubsystem::setTargetPositionStowed);
        // TODO: Add climber extend method call when it is defined
      }
    });

    this.waitForClimbRetractState = extensionStateMachine.registerState(new State<CoordinationLayer>("WaitForClimberRetract") {
      @Override
      protected void periodic(StateMachine<CoordinationLayer> stateMachine, CoordinationLayer world) {
        intake.ifPresent(IntakeSubsystem::setTargetPositionStowed);
        // TODO: Add climber stow method call when it is defined

        // TODO: Add check for whether or not the climber is stowed
        // Assume the climber is stowed if it is disabled
        if (true) {
          finish();
        }
      }
    });


    this.noExtensionState.when(() -> goalExtensionState == ExtensionState.IntakeDeployed, "Goal is IntakeDeployed").transitionTo(intakeDeployedState);
    this.noExtensionState.when(() -> goalExtensionState == ExtensionState.ClimbDeployed, "Goal is ClimbDeployed").transitionTo(climberDeployedState);

    this.intakeDeployedState.when(() -> goalExtensionState != ExtensionState.IntakeDeployed, "Goal is not IntakeDeployed").transitionTo(waitForIntakeRetractState);

    this.waitForIntakeRetractState.whenFinished("Intake finished retracting").transitionTo(noExtensionState);

    this.climberDeployedState.when(() -> goalExtensionState != ExtensionState.ClimbDeployed, "Goal is not ClimbDeployed").transitionTo(waitForClimbRetractState);

    this.waitForClimbRetractState.whenFinished("Climb finished retracting").transitionTo(noExtensionState);

    extensionStateMachine.setState(noExtensionState);
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

    // TODO: Add smart mode climb controls
    Trigger eitherClimbPressed =
        makeTriggerFromButton(controllers.getButton("climbLeft"))
            .or(controllers.getButton("climbRight").getPrimitiveIsPressedSupplier());
    eitherClimbPressed
        .onTrue(
            new InstantCommand(
                () -> {
                  // TODO: Add real climber controls once the climber exists
                  System.out.println("Climb going up!");
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  System.out.println("Climb going down!");
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
        .whileTrue(new RunCommand(this::toggleIntakeDeploy));

    this.isOperatorStowHoodForTrenchPressed =
        controllers.getButton("operatorStowHood").getPrimitiveIsPressedSupplier();

    var won1Supplier = controllers.getButton("operatorWonAuto1").getPrimitiveIsPressedSupplier();
    var won2Supplier = controllers.getButton("operatorWonAuto2").getPrimitiveIsPressedSupplier();
    this.isWonAutoPressed = () -> won1Supplier.getAsBoolean() || won2Supplier.getAsBoolean();

    var lost1Supplier = controllers.getButton("operatorLostAuto1").getPrimitiveIsPressedSupplier();
    var lost2Supplier = controllers.getButton("operatorLostAuto2").getPrimitiveIsPressedSupplier();
    this.isLostAutoPressed = () -> lost1Supplier.getAsBoolean() || lost2Supplier.getAsBoolean();

    makeTriggerFromButton(controllers.getButton("operatorStartShooting"))
        .whileTrue(new RunCommand(this::startShooting));

    this.isForceShootPressed =
        controllers.getButton("operatorForceShoot").getPrimitiveIsPressedSupplier();

    makeTriggerFromButton(controllers.getButton("operatorStopShooting"))
        .whileTrue(new RunCommand(this::stopShooting));

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
      goalExtensionState = ExtensionState.None;
    }
  }

  private void disableAutonomy() {
    autonomyLevel = AutonomyLevel.Manual;
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
  /**
   * This method is like the "periodic" of the CoordinationLayer. It is run by the
   * DependencyOrderedExecutor and is responsible for polling our custom button loop, reading robot
   * goal actions, deciding the best subsystem actions based on the goal robot actions, and applying
   * commands to subsystems. This method must run after subsystems update inputs but before
   * CommandScheduler.run, as it will tell subsystems which commands to apply when their periodic
   * methods run.
   */
  public void coordinateRobotActions() {
    // Poll buttons. This will modify state variables depending on the states of the buttons
    buttonLoop.poll();

    // Handle extension coordination and command intake/climber to their correct positions
    Logger.recordOutput("CoordinationLayer/extensionState", extensionStateMachine.getCurrentState().getName());
    extensionStateMachine.periodic();
    Logger.recordOutput("CoordinationLayer/extensionStateAfter", extensionStateMachine.getCurrentState().getName());

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
