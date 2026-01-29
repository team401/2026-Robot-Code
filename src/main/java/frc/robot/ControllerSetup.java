package frc.robot;

import coppercore.wpilib_interface.DriveWithJoysticks;
import coppercore.wpilib_interface.controllers.Controllers;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

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
  public static void initDriveBindings(Drive drive) {
    var controllers = getControllers();
    /* By making DriveWithJoysticks the default command for the drive subsystem,
     * we ensure that it is run iff no other Command that uses the drive subsystem
     * is activated. */
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            controllers.getAxis("driveX").getSupplier(),
            controllers.getAxis("driveY").getSupplier(),
            controllers.getAxis("driveRotation").getSupplier(),
            JsonConstants.drivetrainConstants.maxLinearSpeed, // type: double (m/s)
            JsonConstants.drivetrainConstants.maxAngularSpeed, // type: double (rad/s)
            JsonConstants.drivetrainConstants.joystickDeadband, // type: double
            JsonConstants.drivetrainConstants.joystickMagnitudeExponent));
  }
}
