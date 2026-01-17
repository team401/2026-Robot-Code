// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.drive.Drive;
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
  private final Optional<LED> led;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    JsonConstants.loadConstants();

    if (JsonConstants.featureFlags.runDrive) {
      drive = Optional.of(InitSubsystems.initDriveSubsystem());
    } else {
      drive = Optional.empty();
    }
    if (JsonConstants.featureFlags.runLEDs) {
      led = Optional.of(InitSubsystems.initLEDs(drive));
    } else {
      led = Optional.empty();
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

    // Configure the button bindings
    configureButtonBindings();
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

  public void periodic() {
    led.ifPresent(led -> led.periodic());
  }
}
