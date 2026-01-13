package frc.robot;

import coppercore.wpilib_interface.Controllers;
import coppercore.wpilib_interface.Controllers.Controller;
import coppercore.wpilib_interface.DriveWithJoysticks;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.function.Supplier;

/** Controllers handles all controller/binding initialization, similar to InitBindings from 2025. */
public class ControllerSetup {
  private ControllerSetup() {}

  private static List<Controller> controllers;

  /** Initialize controllers by loading them from the JSON synced file */
  private static void initControllers() {
    Controllers.synced.setFile(
        JsonConstants.environmentHandler
            .getEnvironmentPathProvider()
            .resolvePath(JsonConstants.operatorConstants.mappingFile));
    Controllers.loadControllers();

    controllers = Controllers.getControllers();
  }

  public static void setupControllers() {
    initControllers();
  }

  /**
   * Given the name of an axis, return a double supplier for that axis's value
   *
   * @param command The name of the axis, e.g. driveX
   * @return A double supplier for that axis, or a supplier that supplies 0.0 in the case of an
   *     unknown axis.
   */
  public static Supplier<Double> getAxis(String command) {
    for (Controllers.Controller controller : controllers) {
      if (controller.hasAxis(command)) {
        return () -> controller.getAxis(command).getAsDouble();
      }
    }

    System.out.println("Could not find Axis with command: " + command);
    return () -> 0.0;
  }

  /**
   * Initialize drive bindings by setting the default command to a DriveWithJoysticks command
   *
   * @param drive A Drive object, the drive subsystem to set the command for
   */
  public static void initDriveBindings(Drive drive) {
    Supplier<Double> xAxis = () -> -getAxis("driveX").get();
    Supplier<Double> yAxis = () -> -getAxis("driveY").get();
    var rotationAxis = getAxis("driveRotation");
    /* By making DriveWithJoysticks the default command for the drive subsystem,
     * we ensure that it is run iff no other Command that uses the drive subsystem
     * is activated. */
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            xAxis,
            yAxis,
            rotationAxis,
            JsonConstants.drivetrainConstants.maxLinearSpeed, // type: double (m/s)
            JsonConstants.drivetrainConstants.maxAngularSpeed, // type: double (rad/s)
            JsonConstants.drivetrainConstants.joystickDeadband, // type: double
            JsonConstants.drivetrainConstants.joystickMagnitudeExponent));
  }
}
