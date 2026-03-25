// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import coppercore.metadata.CopperCoreMetadata;
import coppercore.parameter_tools.json.JSONHandler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.HomingSwitch;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transferroller.TransferRollerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.TotalCurrentCalculator;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
// i helped copilot autocomplete write this
public class RobotContainer {
  private final DependencyOrderedExecutor dependencyOrderedExecutor;

  // Subsystems
  private final Optional<Drive> drive;
  private final Optional<DriveCoordinator> driveCoordinator;
  private final Optional<IntakeSubsystem> intakeSubsystem;
  // Since the RobotContainer doesn't really need a reference to any subsystem except for drive,
  // their references are stored in the CoordinationLayer instead

  private final CoordinationLayer coordinationLayer;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  // JSON jsonHandler
  private JSONHandler jsonHandler;

  public static final ActionKey RUN_COMMAND_SCHEDULER = new ActionKey("CommandScheduler::run");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CopperCoreMetadata.printInfo();

    jsonHandler = JsonConstants.loadConstants(this);
    JsonConstants.featureFlags.logFlags();

    dependencyOrderedExecutor = new DependencyOrderedExecutor();
    dependencyOrderedExecutor.registerAction(
        RUN_COMMAND_SCHEDULER, CommandScheduler.getInstance()::run);
    dependencyOrderedExecutor.addDependencies(
        RUN_COMMAND_SCHEDULER, CoordinationLayer.COORDINATE_ROBOT_ACTIONS);

    coordinationLayer = new CoordinationLayer(dependencyOrderedExecutor);

    if (JsonConstants.featureFlags.runClimber) {
      ClimberSubsystem climber = InitSubsystems.initClimberSubsystem();
      coordinationLayer.setClimber(climber);
    }
    if (JsonConstants.featureFlags.runDrive) {
      Drive drive = InitSubsystems.initDriveSubsystem();
      DriveCoordinator driveCoordinator = new DriveCoordinator(drive);
      this.drive = Optional.of(drive);
      this.driveCoordinator = Optional.of(driveCoordinator);
      coordinationLayer.setDrive(drive);
      coordinationLayer.setDriveCoordinator(driveCoordinator);
      if (JsonConstants.featureFlags.runVision) {
        coordinationLayer.setVisionLocalizer(InitSubsystems.initVisionSubsystem(drive));
      }
    } else {
      drive = Optional.empty();
      driveCoordinator = Optional.empty();
    }

    if (JsonConstants.featureFlags.runHopper) {
      HopperSubsystem hopper = InitSubsystems.initHopperSubsystem();
      coordinationLayer.setHopper(hopper);
    }

    if (JsonConstants.featureFlags.runIndexer) {
      IndexerSubsystem indexer = InitSubsystems.initIndexerSubsystem();
      coordinationLayer.setIndexer(indexer);
    }

    if (JsonConstants.featureFlags.runTransferRoller) {
      TransferRollerSubsystem transferRoller = InitSubsystems.initTransferRollerSubsystem();
      coordinationLayer.setTransferRoller(transferRoller);
    }

    if (JsonConstants.featureFlags.runIntake) {
      IntakeSubsystem intakeSubsystem = InitSubsystems.initIntakeSubsystem();
      coordinationLayer.setIntake(intakeSubsystem);
      dependencyOrderedExecutor.addDependencies(
          RUN_COMMAND_SCHEDULER, CoordinationLayer.UPDATE_INTAKE_DEPENDENCIES);
      this.intakeSubsystem = Optional.of(intakeSubsystem);
    } else {
      this.intakeSubsystem = Optional.empty();
    }

    if (JsonConstants.featureFlags.runHood) {
      HoodSubsystem hood = InitSubsystems.initHoodSubsystem(dependencyOrderedExecutor);
      coordinationLayer.setHood(hood);
      dependencyOrderedExecutor.addDependencies(
          RUN_COMMAND_SCHEDULER, CoordinationLayer.UPDATE_HOOD_DEPENDENCIES);
    }

    if (JsonConstants.featureFlags.runTurret) {
      TurretSubsystem turret = InitSubsystems.initTurretSubsystem(dependencyOrderedExecutor);
      coordinationLayer.setTurret(turret);
      dependencyOrderedExecutor.addDependencies(
          RUN_COMMAND_SCHEDULER, CoordinationLayer.UPDATE_TURRET_DEPENDENCIES);
    }

    if (JsonConstants.featureFlags.runShooter) {
      ShooterSubsystem shooter = InitSubsystems.initShooterSubsystem(dependencyOrderedExecutor);

      coordinationLayer.setShooter(shooter);
    }

    if (JsonConstants.featureFlags.useHomingSwitch) {
      HomingSwitch homingSwitch = InitSubsystems.initHomingSwitch(dependencyOrderedExecutor);
      coordinationLayer.setHomingSwitch(homingSwitch);
    }

    drive.ifPresentOrElse(
        this::createAutoChooser,
        () -> {
          autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        });

    loadAutoCommands();

    // Configure the button bindings
    configureButtonBindings();

    dependencyOrderedExecutor.finalizeSchedule();

    if (Constants.currentMode == Mode.REPLAY) {
      TotalCurrentCalculator.enable();
    }
  }

  public void loadAutoCommands() {
    JsonConstants.autos.loadAllPathPlannerPaths();

    JsonConstants.autos.loadAutoCommands(driveCoordinator.orElse(null), coordinationLayer);

    createAutoChooser(drive.orElse(null));
  }

  public void updateRobotModel() {
    var turretAngle =
        coordinationLayer
            .getTurret()
            .map(TurretSubsystem::getGoalTurretHeading)
            .orElse(Rotation2d.kZero)
            .minus(
                coordinationLayer.getDrive().map(Drive::getPose).orElse(Pose2d.kZero).getRotation())
            .minus(Rotation2d.k180deg);
    Angle hoodAngle =
        coordinationLayer.getHood().map(HoodSubsystem::getCurrentExitPitch).orElse(Radians.zero());
    Angle intakeAngle =
        coordinationLayer
            .getIntake()
            .map(IntakeSubsystem::getCurrentPivotAngle)
            .orElse(Radians.zero());
    Distance climbHeight =
        coordinationLayer.getClimber().map(ClimberSubsystem::getHeightMeters).orElse(Meters.zero());

    var shooterBasePosition =
        new Pose3d(new Translation3d(-0.058, 0.245, 0.37), new Rotation3d(0.0, 0.0, -Math.PI / 2));
    var shooterOffsetFromReferencePoint =
        new Transform3d(0.1045, -0.039, 0, new Rotation3d(0, 0, 0));
    var shooterWithTurretAngle =
        shooterBasePosition
            .plus(shooterOffsetFromReferencePoint)
            .plus(new Transform3d(0, 0, 0, new Rotation3d(turretAngle)))
            .plus(shooterOffsetFromReferencePoint.inverse());
    var hoodOffsetFromShooter =
        new Transform3d(0.1875, -0.04, 0.033, new Rotation3d(Math.PI / 2, 0, 0));
    var hoodOffsetFromReferencePoint = new Transform3d(0, 0.021, 0.101, new Rotation3d(0, 0, 0));

    var intakeOffsetFromReferencePoint =
        new Transform3d(0.352, 0.158, 0.034, new Rotation3d(0, 0, 0));

    // The order of these components are described in
    // advantagekit_config/Robot_401_2026/reference.txt
    Logger.recordOutput(
        "componentPositions",
        new Pose3d[] {
          // Shooter base
          shooterWithTurretAngle,
          // Indexer + hopper
          new Pose3d(
              new Translation3d(0.121, 0.025, 0.0), new Rotation3d(Math.PI / 2, 0.0, Math.PI / 2)),
          // Intake
          new Pose3d(
                  new Translation3d(0.35, 0.0, 0.0), new Rotation3d(Math.PI / 2, 0.0, -Math.PI / 2))
              .plus(intakeOffsetFromReferencePoint)
              .plus(new Transform3d(0, 0, 0, new Rotation3d(intakeAngle.in(Radians), 0, 0)))
              .plus(intakeOffsetFromReferencePoint.inverse()),
          // Turret
          new Pose3d(
              new Translation3d(-0.099, 0.138, 0.331),
              new Rotation3d(Math.PI / 2, 0.0, -Math.PI / 2)),
          // Climb
          new Pose3d(
              new Translation3d(-0.180, -0.218, -0.190 + climbHeight.in(Meters)),
              new Rotation3d(Math.PI / 2, 0.0, Math.PI)),
          // Hood
          shooterWithTurretAngle
              .plus(hoodOffsetFromShooter)
              .plus(hoodOffsetFromReferencePoint)
              .plus(
                  new Transform3d(
                      0,
                      0,
                      0,
                      new Rotation3d(
                          -hoodAngle.in(Radians)
                              + Math.toRadians(
                                  50.0), // This is an estimate of the hood offset angle
                          0,
                          0)))
              .plus(hoodOffsetFromReferencePoint.inverse())
        });
  }

  /*
   * Process any pending HTTP requests from the tuning server
   * Calling this method in periodic ensures that the updates to the JSONConstants
   * objects are done within the context of the robot main thread, thus avoiding
   * race conditions.
   */
  void processHTTPRequests() {
    if (JsonConstants.featureFlags.useTuningServer) {
      long startTimeUs = RobotController.getFPGATime();

      jsonHandler.drainQueuedHttpActions();

      long endTimeUs = RobotController.getFPGATime();
      if (JsonConstants.featureFlags.logPeriodicTiming) {
        Logger.recordOutput("PeriodicTime/httpRequestsMs", (endTimeUs - startTimeUs) / 1000.0);
      }
    }
  }

  /**
   * Create the auto chooser and publish it to network tables
   *
   * @param drive The Drive instance to use for the auto chooser
   */
  private void createAutoChooser(Drive drive) {
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Set up SysId routines
    if (drive != null) {
      autoChooser.addOption(
          "Drive Wheel Radius Characterization",
          DriveCoordinatorCommands.wrapCommand(
              driveCoordinator.get(), DriveCommands.wheelRadiusCharacterization(drive)));
      autoChooser.addOption(
          "Drive Simple FF Characterization",
          DriveCoordinatorCommands.wrapCommand(
              driveCoordinator.get(), DriveCommands.feedforwardCharacterization(drive)));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          DriveCoordinatorCommands.wrapCommand(
              driveCoordinator.get(), drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)",
          DriveCoordinatorCommands.wrapCommand(
              driveCoordinator.get(), drive.sysIdDynamic(SysIdRoutine.Direction.kForward)));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)",
          DriveCoordinatorCommands.wrapCommand(
              driveCoordinator.get(), drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)));
    }

    for (var auto : JsonConstants.autos.autoCommands.entrySet()) {
      autoChooser.addOption(auto.getKey(), auto.getValue());
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    coordinationLayer.initBindings();

    // Default command, normal field-relative drive
    driveCoordinator.ifPresent(
        driveCoordinator -> {
          drive.ifPresent(
              (drive) -> {
                ControllerSetup.initDriveBindings(driveCoordinator, drive);
              });
        });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Get the DependencyOrderedExecutor instance used by the RobotContainer and all subsystems
   *
   * <p>This method exists so that Robot can access the DOE in its periodic method
   *
   * @return The DependencyOrderedExecutor instance
   */
  public DependencyOrderedExecutor getDependencyOrderedExecutor() {
    return dependencyOrderedExecutor;
  }

  public Drive getDriveSubsystem() {
    return drive.orElse(null);
  }
}
