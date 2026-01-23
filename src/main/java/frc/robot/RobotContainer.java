// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.ShooterCalculations.ShotInfo;
import frc.robot.ShooterCalculations.ShotType;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
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
  private final Optional<Void> homingSwitch = Optional.empty();

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  public static final ActionKey RUN_COMMAND_SCHEDULER = new ActionKey("CommandScheduler::run");
  // TODO: Decide if these belong in RobotContainer or if our coordination layer should have its own
  // class
  public static final ActionKey UPDATE_TURRET_DEPENDENCIES =
      new ActionKey("RobotContainer::updateTurretDependencies");
  public static final ActionKey RUN_SHOT_CALCULATOR =
      new ActionKey("RobotContainer::runShotCalculator");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    JsonConstants.loadConstants();

    var dependencyOrderedExecutor = DependencyOrderedExecutor.getDefaultInstance();
    dependencyOrderedExecutor.registerAction(
        RUN_COMMAND_SCHEDULER, CommandScheduler.getInstance()::run);
    dependencyOrderedExecutor.registerAction(
        UPDATE_TURRET_DEPENDENCIES, this::updateTurretDependencies);
    dependencyOrderedExecutor.registerAction(RUN_SHOT_CALCULATOR, this::runShotCalculator);
    dependencyOrderedExecutor.addDependencies(RUN_COMMAND_SCHEDULER, RUN_SHOT_CALCULATOR);

    TestModeManager.init();

    if (JsonConstants.featureFlags.runDrive) {
      drive = Optional.of(InitSubsystems.initDriveSubsystem());
    } else {
      drive = Optional.empty();
    }

    if (JsonConstants.featureFlags.runTurret) {
      turret = Optional.of(InitSubsystems.initTurretSubsystem());
      dependencyOrderedExecutor.addDependencies(RUN_SHOT_CALCULATOR, TurretSubsystem.UPDATE_INPUTS);
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

  // Subsystem dependency updates
  /** Update the turret subsystem on the state of the homing switch. */
  private void updateTurretDependencies() {
    // TODO: Add other subsystems that require homing, like the hood and the intake.
    // TODO: Add actual check for whether homing switch is present once it is designed
    // This check is written to mimic how the actual check would look when dealing with a homing
    // switch that was an actual "mini-subsystem"
    turret.ifPresent(
        turret -> turret.setIsHomingSwitchPressed(homingSwitch.map(s -> false).orElse(false)));
  }

  /**
   * Runs the shot calculator and logs the resulting trajectories for debugging. Eventually, this
   * method should also command the subsystems to take their actinos.
   *
   * <p>This should likely go into some kind of coordination layer class.
   */
  public void runShotCalculator() {
    drive.ifPresent(
        driveInstance -> {
          Translation3d hubTranslation =
              new Translation3d(
                  Units.inchesToMeters(182.11),
                  Units.inchesToMeters(158.84),
                  Units.inchesToMeters(72 - 8));
          // Placeholder passing "example" translation
          Translation3d passingTargetTranslation = new Translation3d(2.0, 1.0, 0.0);

          // Pick either passing target or hub here.
          Translation3d goalTranslation = hubTranslation;

          Pose2d robotPose = driveInstance.getPose();
          ChassisSpeeds fieldCentricSpeeds =
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  driveInstance.getChassisSpeeds(), robotPose.getRotation());

          Translation2d goalXYPlane =
              new Translation2d(goalTranslation.getX(), goalTranslation.getY());
          Logger.recordOutput(
              "Superstructure/ShotXYDistanceMeters",
              robotPose.getTranslation().getDistance(goalXYPlane));

          Optional<ShotInfo> maybeShot =
              ShooterCalculations.calculateMovingShot(
                  new Pose3d(robotPose).getTranslation(),
                  goalTranslation,
                  fieldCentricSpeeds,
                  MetersPerSecond.of(10.64),
                  ShotType.HIGH,
                  Optional.empty());

          Optional<ShotInfo> maybeStaticShot =
              ShooterCalculations.calculateStationaryShot(
                  new Pose3d(robotPose).getTranslation(),
                  goalTranslation,
                  MetersPerSecond.of(10.64),
                  ShotType.HIGH);

          final int trajectoryPointsPerMeter = 4;

          maybeShot.ifPresent(
              shot -> {
                Translation3d[] shotTrajectory =
                    shot.projectMotion(
                        10.64,
                        driveInstance.getPose(),
                        fieldCentricSpeeds,
                        trajectoryPointsPerMeter);
                Logger.recordOutput("Superstructure/Shot", shot);
                Logger.recordOutput(
                    "Superstructure/EffectiveTrajectory",
                    shot.projectMotion(
                        10.64,
                        driveInstance.getPose(),
                        new ChassisSpeeds(),
                        trajectoryPointsPerMeter));
                Logger.recordOutput("Superstructure/ShotTrajectory", shotTrajectory);
                Logger.recordOutput(
                    "Superstructure/ShotErrorMeters",
                    shotTrajectory[shotTrajectory.length - 1].getDistance(goalTranslation));
              });

          maybeStaticShot.ifPresent(
              shot -> {
                Logger.recordOutput(
                    "Superstructure/StaticTrajectory",
                    shot.projectMotion(
                        10.64, robotPose, fieldCentricSpeeds, trajectoryPointsPerMeter));
              });
        });
  }
}
