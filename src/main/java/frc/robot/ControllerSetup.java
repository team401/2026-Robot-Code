package frc.robot;

import coppercore.wpilib_interface.DriveWithJoysticks;
import coppercore.wpilib_interface.controllers.Controllers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.drive.DriveCoordinator.DriveAction;

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

    // Sample climb pose; This should be moved to a constants file (see #34) but it's located on the
    // red side of the field against the left upright of the tower.
    Pose2d targetPose = new Pose2d(14.968, 3.9, new Rotation2d(Math.toRadians(-90.0)));

    controllers
        .getButton("testClimbDrive")
        .getTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  driveCoordinator.setLinearTargetPose(targetPose);
                  driveCoordinator.setDriveAction(DriveAction.LinearDriveToPose);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  driveCoordinator.setDriveAction(DriveAction.DriveWithJoysticks);
                }));
  }
}
