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
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Optional<Drive> drive;
  private final Optional<TurretSubsystem> turret;
  private final Optional<HoodSubsystem> hood;
  private final Optional<Void> homingSwitch = Optional.empty();

  private final CoordinationLayer coordinationLayer;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  public static final ActionKey RUN_COMMAND_SCHEDULER = new ActionKey("CommandScheduler::run");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CopperCoreMetadata.printInfo();

    JsonConstants.loadConstants();

    var dependencyOrderedExecutor = DependencyOrderedExecutor.getDefaultInstance();
    dependencyOrderedExecutor.registerAction(
        RUN_COMMAND_SCHEDULER, CommandScheduler.getInstance()::run);
    dependencyOrderedExecutor.addDependencies(
        RUN_COMMAND_SCHEDULER, CoordinationLayer.RUN_SHOT_CALCULATOR);
    // Homing switch must be updated before running subsystem periodics because certain state
    // machines will take action during periodic based on its state.
    dependencyOrderedExecutor.addDependencies(
        RUN_COMMAND_SCHEDULER, CoordinationLayer.UPDATE_HOMING_SWITCH);

    if (JsonConstants.featureFlags.runDrive) {
      drive = Optional.of(InitSubsystems.initDriveSubsystem());
    } else {
      drive = Optional.empty();
    }

    if (JsonConstants.featureFlags.runHood) {
      hood = Optional.of(InitSubsystems.initHoodSubsystem());
      dependencyOrderedExecutor.addDependencies(
          RUN_COMMAND_SCHEDULER, CoordinationLayer.UPDATE_HOOD_DEPENDENCIES);
    } else {
      hood = Optional.empty();
    }

    if (JsonConstants.featureFlags.runTurret) {
      turret = Optional.of(InitSubsystems.initTurretSubsystem());
      dependencyOrderedExecutor.addDependencies(
          RUN_COMMAND_SCHEDULER, CoordinationLayer.UPDATE_TURRET_DEPENDENCIES);
    } else {
      turret = Optional.empty();
    }

    drive.ifPresentOrElse(
        drive -> {
          // TODO: Stop using pathplanner AutoBuilder
          // Set up auto routines
          autoChooser =
              new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

          // Set up SysId routines
          autoChooser.addOption(
              "Drive Wheel Radius Characterization",
              DriveCommands.wheelRadiusCharacterization(drive));
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
        },
        () -> {
          autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        });

    coordinationLayer = new CoordinationLayer(drive, turret, hood, homingSwitch);

    // Configure the button bindings
    configureButtonBindings();

    dependencyOrderedExecutor.finalizeSchedule();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // TODO: Create a robust and clean input/action layer.
    ControllerSetup.setupControllers();

    // Default command, normal field-relative drive
    drive.ifPresent(drive -> ControllerSetup.initDriveBindings(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
