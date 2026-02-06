// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import coppercore.metadata.CopperCoreMetadata;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.HomingSwitch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.external.hammerheads5000.FuelSim;
import java.util.Optional;
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

  protected final Optional<FuelSim> fuelSim;

  // Subsystems
  private final Optional<Drive> drive;
  private final Optional<DriveCoordinator> driveCoordinator;
  // Since the RobotContainer doesn't really need a reference to any subsystem except for drive,
  // their references are stored in the CoordinationLayer instead

  private final CoordinationLayer coordinationLayer;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  public static final ActionKey RUN_COMMAND_SCHEDULER = new ActionKey("CommandScheduler::run");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CopperCoreMetadata.printInfo();

    JsonConstants.loadConstants();

    dependencyOrderedExecutor = new DependencyOrderedExecutor();
    dependencyOrderedExecutor.registerAction(
        RUN_COMMAND_SCHEDULER, CommandScheduler.getInstance()::run);
    // Homing switch must be updated before running subsystem periodics because certain state
    // machines will take action during periodic based on its state.
    dependencyOrderedExecutor.addDependencies(
        RUN_COMMAND_SCHEDULER, CoordinationLayer.RUN_SHOT_CALCULATOR);

    if (Constants.currentMode == Mode.SIM) {
      var fuelSimInstance = new FuelSim("FuelSim");
      fuelSim = Optional.of(fuelSimInstance);
      fuelSimInstance.start();
    } else {
      fuelSim = Optional.empty();
    }

    coordinationLayer = new CoordinationLayer(this, dependencyOrderedExecutor);

    if (JsonConstants.featureFlags.runDrive) {
      Drive drive = InitSubsystems.initDriveSubsystem();
      DriveCoordinator driveCoordinator = new DriveCoordinator(drive);
      this.drive = Optional.of(drive);
      this.driveCoordinator = Optional.of(driveCoordinator);
      coordinationLayer.setDrive(drive);
      coordinationLayer.setDriveCoordinator(driveCoordinator);
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

    if (JsonConstants.featureFlags.runIndexer || JsonConstants.featureFlags.runHopper) {
      // Ensure that demo modes are run before subsystem periodics if either of the 2 subsystems
      // that have demo modes are active
      dependencyOrderedExecutor.addDependencies(
          RUN_COMMAND_SCHEDULER, CoordinationLayer.RUN_DEMO_MODES);
    }

    drive.ifPresentOrElse(
        this::createAutoChooser,
        () -> {
          autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        });

    // Configure the button bindings
    configureButtonBindings();

    dependencyOrderedExecutor.finalizeSchedule();
  }

  /**
   * Create the auto chooser and publish it to network tables
   *
   * @param drive The Drive instance to use for the auto chooser
   */
  private void createAutoChooser(Drive drive) {
    // TODO: Stop using pathplanner AutoBuilder
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // TODO: Create a robust and clean input/action layer.

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
}
