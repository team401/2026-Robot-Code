package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import coppercore.wpilib_interface.DriveWithJoysticks;
import coppercore.wpilib_interface.controllers.Controllers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;
import frc.robot.subsystems.drive.DriveCoordinator.ClimbLocations;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * The ControllerSetup class handles all controller/binding initialization, similar to InitBindings
 * from 2025.
 *
 * <p>Before calling any individual subsystem button initialization, setupControllers must be called
 * to load the bindings file from JSON.
 */
public class ControllerSetup {
  private ControllerSetup() {}

  private static Controllers getControllers() {
    return JsonConstants.controllers;
  }

  /**
   * Initialize drive bindings by setting the default command to a DriveWithJoysticks command
   *
   * @param drive A Drive object, the drive subsystem to set the command for
   */
  public static void initDriveBindings(DriveCoordinator driveCoordinator, Drive drive) {
    var controllers = getControllers();

    var joystickDriveCommand =
        new DriveWithJoysticks(
            drive,
            controllers.getAxis("driveX").getSupplier(),
            controllers.getAxis("driveY").getSupplier(),
            controllers.getAxis("driveRotation").getSupplier(),
            JsonConstants.driveConstants.maxLinearSpeed, // type: double (m/s)
            JsonConstants.driveConstants.maxAngularSpeed, // type: double (rad/s)
            JsonConstants.driveConstants.joystickDeadband, // type: double
            JsonConstants.driveConstants.joystickMagnitudeExponent // type: double
            );

    driveCoordinator.setDriveWithJoysticksCommand(joystickDriveCommand);

    controllers
        .getButton("testClimbDrive").getTrigger()
        .onTrue(
            driveCoordinator.createInstantCommandToSetCurrent(
                driveCoordinator.getDriveToClimbCommand(ClimbLocations.LeftClimbLocation)))
        .onFalse(
            driveCoordinator.createInstantCommandToCancelCommand());

    var combinedAutoPilot =
        DriveCoordinatorCommands.autoPilotToTargetsCommand(
            driveCoordinator,
            new APProfile(new APConstraints().withAcceleration(5.0).withJerk(3.0))
                .withErrorXY(Meters.of(0.2))
                .withErrorTheta(Degrees.of(5))
                .withBeelineRadius(Meters.of(0.2)),
            new APTarget(new Pose2d(12.6, 7.4, new Rotation2d(Math.toRadians(90))))
                .withEntryAngle(new Rotation2d(Degrees.of(180)))
                .withVelocity(MetersPerSecond.of(3.0).in(MetersPerSecond)),
            new APTarget(new Pose2d(11.4, 7.4, new Rotation2d(Math.toRadians(90))))
                .withEntryAngle(new Rotation2d(Degrees.of(180)))
                .withVelocity(MetersPerSecond.of(3.0).in(MetersPerSecond)),
            new APTarget(new Pose2d(8.2, 4, new Rotation2d(Math.toRadians(90))))
                .withEntryAngle(new Rotation2d(Degrees.of(270))));

    controllers
        .getButton("testGoToAllianceCenter").getTrigger()
        .onTrue(
            driveCoordinator.createInstantCommandToSetCurrent(combinedAutoPilot))
        .onFalse(
            driveCoordinator.createInstantCommandToCancelCommand());
  }

  public static void initIntakeBindings(IntakeSubsystem intakeSubsystem) {
    // These are just temporary bindings for testing the intake subsystem

    var controllers = getControllers();

    controllers
        .getButton("intakePivotUp")
        .getTrigger()
        .onTrue(new InstantCommand(intakeSubsystem::setTargetPositionStowed));

    controllers
        .getButton("intakePivotDown")
        .getTrigger()
        .onTrue(new InstantCommand(intakeSubsystem::setTargetPositionIntaking));

    controllers
        .getButton("runIntakeRollers")
        .getTrigger()
        .whileTrue(
            new InstantCommand(
                () -> intakeSubsystem.runRollers(JsonConstants.intakeConstants.intakeRollerSpeed)))
        .onFalse(new InstantCommand(intakeSubsystem::stopRollers));
  }
}
