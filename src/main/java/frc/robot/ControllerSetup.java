package frc.robot;

import coppercore.wpilib_interface.DriveWithJoysticks;
import coppercore.wpilib_interface.controllers.Controllers;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;

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

  //   private static LoggedNetworkNumber trenchEndVelocityMps =
  //       new LoggedNetworkNumber("DriveCoordinator/TrenchConstraints/EndVelocityMps", 4.0);
  //   private static LoggedNetworkNumber trenchAccelMpsSquared =
  //       new LoggedNetworkNumber("DriveCoordinator/TrenchConstraints/AccelMps2", 6.0);
  //   private static LoggedNetworkNumber trenchJerkMpsCubed =
  //       new LoggedNetworkNumber("DriveCoordinator/TrenchConstraints/JerkMps3", 1.0);

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

    // Temporary testing setup
    // controllers
    //     .getButton("testClimbDrive")
    //     .getTrigger()
    //     .onTrue(
    //         driveCoordinator.createInstantCommandToSetCurrent(
    //             driveCoordinator.getDriveToClimbCommand(ClimbLocations.LeftClimbLocation)))
    //     // .onTrue(
    //     //    driveCoordinator.createInstantCommandToSetCurrent(autoCommand))
    //     // .onTrue(autoCommand)
    //     .onFalse(driveCoordinator.createInstantCommandToCancelCommand());

    // Pose2d pose1 = new Pose2d(12.6, 7.33, new Rotation2d(Math.toRadians(90)));

    // Pose2d pose2 = new Pose2d(11.4, 7.33, new Rotation2d(Math.toRadians(90)));

    // controllers
    //     .getButton("testGoToAllianceCenter")
    //     .getTrigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               var constraints =
    //                   new APConstraints()
    //                       .withAcceleration(trenchAccelMpsSquared.get())
    //                       .withJerk(trenchJerkMpsCubed.get());

    //               var profile =
    //                   new APProfile(constraints)
    //                       .withErrorXY(Meters.of(0.2))
    //                       .withErrorTheta(Degrees.of(5))
    //                       .withBeelineRadius(Meters.of(0.2));

    //               double endVelocityMps = trenchEndVelocityMps.get();
    //               var target1 =
    //                   new APTarget(pose1)
    //                       .withEntryAngle(new Rotation2d(Degrees.of(180)))
    //                       .withVelocity(MetersPerSecond.of(endVelocityMps).in(MetersPerSecond));

    //               var target2 =
    //                   new APTarget(pose2)
    //                       .withEntryAngle(new Rotation2d(Degrees.of(180)))
    //                       .withVelocity(MetersPerSecond.of(endVelocityMps).in(MetersPerSecond));

    //               Autopilot autoPilot = new Autopilot(profile);
    //               var command1 =
    //                   new XBasedAutoPilotCommand(
    //                       driveCoordinator,
    //                       autoPilot,
    //                       target1,
    //                       DriveCoordinatorCommands.createDefaultAutoPilotHeadingController());
    //               var command2 =
    //                   new XBasedAutoPilotCommand(
    //                       driveCoordinator,
    //                       autoPilot,
    //                       target2,
    //                       DriveCoordinatorCommands.createDefaultAutoPilotHeadingController());
    //               driveCoordinator.setCurrentCommand(command1.andThen(command2));
    //             }))
    //     .onFalse(driveCoordinator.createInstantCommandToCancelCommand());
  }
}
