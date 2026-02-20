package frc.robot;

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
import frc.robot.auto.AutoAction.AutoActionData;
import frc.robot.auto.AutoManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.subsystems.drive.DriveCoordinatorCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

  private static LoggedNetworkNumber trenchEndVelocityMps =
      new LoggedNetworkNumber("DriveCoordinator/TrenchConstraints/EndVelocityMps", 4.0);
  private static LoggedNetworkNumber trenchAccelMpsSquared =
      new LoggedNetworkNumber("DriveCoordinator/TrenchConstraints/AccelMps2", 6.0);
  private static LoggedNetworkNumber trenchJerkMpsCubed =
      new LoggedNetworkNumber("DriveCoordinator/TrenchConstraints/JerkMps3", 1.0);

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

    var autoAction = AutoManager.loadAuto("testAuto.json");
    var autoActionData = new AutoActionData(driveCoordinator);
    var autoCommand = autoAction.toCommand(autoActionData);
    // Temporary testing setup
    controllers
        .getButton("testClimbDrive")
        .getTrigger()
        // .onTrue(
        // driveCoordinator.createInstantCommandToSetCurrent(
        // driveCoordinator.getDriveToClimbCommand(ClimbLocations.LeftClimbLocation)))
        //.onTrue(
        //    driveCoordinator.createInstantCommandToSetCurrent(autoCommand))
        .onTrue(autoCommand)
        .onFalse(driveCoordinator.createInstantCommandToCancelCommand());

    Pose2d pose1 = new Pose2d(12.6, 7.33, new Rotation2d(Math.toRadians(90)));

    Pose2d pose2 = new Pose2d(11.4, 7.33, new Rotation2d(Math.toRadians(90)));

    controllers
        .getButton("testGoToAllianceCenter")
        .getTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  var constraints =
                      new APConstraints()
                          .withAcceleration(trenchAccelMpsSquared.get())
                          .withJerk(trenchJerkMpsCubed.get());

                  var profile =
                      new APProfile(constraints)
                          .withErrorXY(Meters.of(0.2))
                          .withErrorTheta(Degrees.of(5))
                          .withBeelineRadius(Meters.of(0.2));

                  double endVelocityMps = trenchEndVelocityMps.get();
                  var target1 =
                      new APTarget(pose1)
                          .withEntryAngle(new Rotation2d(Degrees.of(180)))
                          .withVelocity(MetersPerSecond.of(endVelocityMps).in(MetersPerSecond));

                  var target2 =
                      new APTarget(pose2)
                          .withEntryAngle(new Rotation2d(Degrees.of(180)))
                          .withVelocity(MetersPerSecond.of(endVelocityMps).in(MetersPerSecond));

                  var target3 =
                      new APTarget(new Pose2d(8.2, 4, new Rotation2d(Math.toRadians(90))))
                          .withEntryAngle(new Rotation2d(Degrees.of(270)));

                  driveCoordinator.setCurrentCommand(
                      DriveCoordinatorCommands.autoPilotToTargetsCommand(
                          driveCoordinator, profile, target1, target2, target3));
                }))
        .onFalse(driveCoordinator.createInstantCommandToCancelCommand());
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
