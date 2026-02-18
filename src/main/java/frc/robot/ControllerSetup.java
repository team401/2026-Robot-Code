package frc.robot;

import coppercore.wpilib_interface.DriveWithJoysticks;
import coppercore.wpilib_interface.controllers.Controllers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.drive.DriveCoordinatorCommands.LinearDriveGoal;
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
        .getButton("testClimbDrive")
        .getTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  driveCoordinator.driveToClimbLocation(
                      DriveCoordinator.ClimbLocations.LeftClimbLocation);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  driveCoordinator.cancelCurrentCommand();
                }));

    Pose2d targetPose = new Pose2d(13, 3.9, new Rotation2d(Math.toRadians(-90.0)));

    LinearDriveGoal linearDriveGoal =
        LinearDriveGoal.toPose(targetPose);

    controllers
        .getButton("testGoToAllianceCenter")
        .getTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  driveCoordinator.linearDriveToGoal(linearDriveGoal);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  driveCoordinator.cancelCurrentCommand();
                }));
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
