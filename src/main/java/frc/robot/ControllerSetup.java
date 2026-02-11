package frc.robot;

import coppercore.wpilib_interface.DriveWithJoysticks;
import coppercore.wpilib_interface.controllers.Controllers;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * The ControllerSetup class handles all controller/binding initialization, similar to InitBindings
 * from 2025.
 *
 * <p>Before calling any individual subsystem button initialization, setupControllers must be called
 * to load the bindings file from JSON.
 *
 * <p>Our controllers-xbox file adds button shorthands for the paddles on the back. These paddles
 * are mapped to:
 *
 * <p>{@code "buttonShorthands": { "topLeftPaddle": 7, "topRightPaddle": 8, "bottomLeftPaddle": 9,
 * "bottomRightPaddle": 10 }, }
 *
 * <ul>
 *   <li>Top left paddle: Back
 *   <li>Top right paddle: Start
 *   <li>Bottom left paddle: Left Stick Press
 *   <li>Bottom right paddle: Right Stick Press
 * </ul>
 *
 * TODO: Add bindings that map left bumper 2 (L3) to dpad down and right bumper 2 (R3) to pad right.
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
    driveCoordinator.createStateMachine(
        new DriveWithJoysticks(
            drive,
            controllers.getAxis("driveX").getSupplier(),
            controllers.getAxis("driveY").getSupplier(),
            controllers.getAxis("driveRotation").getSupplier(),
            JsonConstants.driveConstants.maxLinearSpeed, // type: double (m/s)
            JsonConstants.driveConstants.maxAngularSpeed, // type: double (rad/s)
            JsonConstants.driveConstants.joystickDeadband, // type: double
            JsonConstants.driveConstants.joystickMagnitudeExponent));
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
