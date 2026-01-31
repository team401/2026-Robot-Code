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
    /* By making DriveWithJoysticks the default command for the drive subsystem,
     * we ensure that it is run iff no other Command that uses the drive subsystem
     * is activated. */
    driveCoordinator.setDriveWithJoysticksCommand(
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
}
