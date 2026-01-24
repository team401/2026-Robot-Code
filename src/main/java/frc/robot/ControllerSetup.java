package frc.robot;

import coppercore.wpilib_interface.Controllers;
import coppercore.wpilib_interface.Controllers.Controller;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import java.util.List;
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

  private static List<Controller> controllers;

  /** Initialize operator interfae/controllers by loading their configurations from JSON. */
  public static void setupControllers() {
    Controllers.synced.setFile(
        JsonConstants.environmentHandler
            .getEnvironmentPathProvider()
            .resolvePath(JsonConstants.operatorConstants.controllerBindingsFile));
    Controllers.loadControllers();

    controllers = Controllers.getControllers();
  }

  /**
   * Given the name of an axis, return a double supplier for that axis's value
   *
   * @param axisName The name of the axis, e.g. driveX
   * @return A double supplier for that axis, or a supplier that supplies 0.0 in the case of an
   *     unknown axis.
   */
  private static Supplier<Double> getAxis(String axisName) {
    for (Controllers.Controller controller : controllers) {
      if (controller.hasAxis(axisName)) {
        return () -> controller.getAxis(axisName).getAsDouble();
      }
    }

    System.out.println("Could not find Axis with command: " + axisName);
    return () -> 0.0;
  }

  /**
   * Gets a Trigger object for a button based on that button's name in the JSON controller config.
   *
   * @param buttonName The name of the button, as seen in the JSON file.
   * @return A Trigger that wraps the state of that button.
   */
  private static Trigger getTriggerForButton(String buttonName) {
    for (Controllers.Controller controller : controllers) {
      if (controller.hasButton(buttonName)) {
        return controller.getButton(buttonName);
      }
    }

    System.out.println("Could not find button with name/command: " + buttonName);
    return new Trigger(() -> false);
  }

  /**
   * Initialize drive bindings by setting the default command to a DriveWithJoysticks command
   *
   * @param drive A Drive object, the drive subsystem to set the command for
   */
  public static void initDriveBindings(DriveCoordinator driveCoordinator, Drive drive) {
    Supplier<Double> xAxis = () -> -getAxis("driveX").get();
    Supplier<Double> yAxis = () -> -getAxis("driveY").get();
    var rotationAxis = getAxis("driveRotation");
    /* By making DriveWithJoysticks the default command for the drive subsystem,
     * we ensure that it is run iff no other Command that uses the drive subsystem
     * is activated. */
    driveCoordinator.setDriveWithJoysticksCommand(
        new DriveWithJoysticks(
            drive,
            xAxis,
            yAxis,
            rotationAxis,
            JsonConstants.driveConstants.maxLinearSpeed, // type: double (m/s)
            JsonConstants.driveConstants.maxAngularSpeed, // type: double (rad/s)
            JsonConstants.driveConstants.joystickDeadband, // type: double
            JsonConstants.driveConstants.joystickMagnitudeExponent));
    Transform2d offset = new Transform2d(0.0, 3.0, new Rotation2d(Math.toRadians(180)));
    getTriggerForButton("linearDrive")
        .onTrue(
            new InstantCommand(
                () -> {
                  driveCoordinator.setDriveAction(DriveCoordinator.DriveAction.LinearDriveToPose);
                  driveCoordinator.setLinearTargetPose(drive.getPose().plus(offset));
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  driveCoordinator.setDriveAction(DriveCoordinator.DriveAction.DriveWithJoysticks);
                }));
  }
}
