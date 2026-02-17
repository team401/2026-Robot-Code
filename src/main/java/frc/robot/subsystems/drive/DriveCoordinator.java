package frc.robot.subsystems.drive;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.control_methods.DisabledDrive;
import frc.robot.subsystems.drive.control_methods.DriveControlMethod;
import frc.robot.subsystems.drive.control_methods.JoystickDrive;
import frc.robot.subsystems.drive.control_methods.LinearDrive;
import frc.robot.subsystems.drive.states.DriveToClimbState;
import frc.robot.subsystems.drive.states.DriveWithJoysticksState;
import frc.robot.subsystems.drive.states.DrivingLinearPathState;
import frc.robot.util.LoggedTunablePIDGains;
import frc.robot.util.TestModeManager;
import org.littletonrobotics.junction.Logger;

// Copilot used to help write this class

// States currently are more so what is drive trying to do
// And Control Methods are more so how drive is trying to do it

public class DriveCoordinator extends SubsystemBase {

  public enum DriveAction {
    DriveWithJoysticks,
    DriveLinearPath,
    DriveToClimb
  }

  public enum ClimbLocations {
    LeftClimbLocation,
    RightClimbLocation
  }

  StateMachine<DriveCoordinator> driveStateMachine;
  public Drive drive;

  private DriveWithJoysticksState driveWithJoysticksState;
  private DrivingLinearPathState linearDriveToPoseState;
  private DriveToClimbState driveToClimbState;

  public DriveControlMethod DISABLED_DRIVE;
  public LinearDrive LINEAR_DRIVE;
  public DriveControlMethod JOYSTICK_DRIVE;

  private DriveControlMethod currentControlMethod;

  public void setControlMethod(DriveControlMethod controlMethod) {
    if (controlMethod == currentControlMethod) {
      return; // No change in control method, so do nothing
    }
    if (currentControlMethod != null) {
      currentControlMethod.deactivateControl();
    }
    currentControlMethod = controlMethod;
    if (currentControlMethod != null) {
      currentControlMethod.activateControl();
    }
  }

  public void initializeJoyStickDriveControl(DriveWithJoysticks command) {
    JOYSTICK_DRIVE = new JoystickDrive(drive, command, "DriveCoordinator");
  }

  public void initializeControlMethods(Drive drive) {
    DISABLED_DRIVE = new DisabledDrive(drive, "DriveCoordinator");
    LINEAR_DRIVE = new LinearDrive(drive, "DriveCoordinator");
    // Initialize to disabled drive until joystick drive is initialized
    JOYSTICK_DRIVE = DISABLED_DRIVE;

    setControlMethod(DISABLED_DRIVE);

    driveStateMachine = new StateMachine<>(this);
  }

  // Test Mode
  final TestModeManager<TestMode> testModeManager = new TestModeManager<>("Drive", TestMode.class);

  LoggedTunablePIDGains steerGains;
  LoggedTunablePIDGains driveGains;

  public void initializeTestMode() {
    steerGains =
        new LoggedTunablePIDGains(
            "DriveCoordinatorTunables/SteerGains",
            JsonConstants.driveConstants.steerGains.asArray());
    driveGains =
        new LoggedTunablePIDGains(
            "DriveCoordinatorTunables/DriveGains",
            JsonConstants.driveConstants.driveGains.asArray());
  }

  public DriveCoordinator(Drive drive) {
    this.drive = drive;
    // drive.setDriveGains(JsonConstants.driveConstants.driveGains);
    // drive.setSteerGains(JsonConstants.driveConstants.steerGains);

    initializeControlMethods(drive);

    linearDriveToPoseState = driveStateMachine.registerState(new DrivingLinearPathState());
    driveWithJoysticksState = driveStateMachine.registerState(new DriveWithJoysticksState());
    driveToClimbState = driveStateMachine.registerState(new DriveToClimbState());

    linearDriveToPoseState.whenFinished().transitionTo(driveWithJoysticksState);

    // TODO: add more climb states for more precise climb line up
    driveToClimbState.whenFinished().transitionTo(driveWithJoysticksState);

    driveStateMachine.setState(driveWithJoysticksState);
  }

  public void setDriveAction(DriveAction action) {
    switch (action) {
      case DriveWithJoysticks:
        driveStateMachine.setState(driveWithJoysticksState);
        break;
      case DriveLinearPath:
        driveStateMachine.setState(linearDriveToPoseState);
        break;
      case DriveToClimb:
        driveStateMachine.setState(driveToClimbState);
        break;
    }
  }

  public void linearDriveToPose(Pose2d pose) {
    linearDriveToPoseState.setCommand(new LinearDrive.LinearDriveCommand(pose));
    setDriveAction(DriveAction.DriveLinearPath);
  }

  public void followLinearDriveCommand(LinearDrive.LinearDriveCommand command) {
    linearDriveToPoseState.setCommand(command);
    setDriveAction(DriveAction.DriveLinearPath);
  }

  public void driveToClimbLocation(ClimbLocations climbLocation) {
    driveToClimbState.setClimbLocation(climbLocation);
    setDriveAction(DriveAction.DriveToClimb);
  }

  @Override
  public void periodic() {

    if (testModeManager.isInTestMode()) {
      testPeriodic();
    }

    driveStateMachine.periodic();

    // There should always be a current control method, but just in case, we can check for null
    // before calling periodic on it
    if (currentControlMethod != null) {
      currentControlMethod.periodic();
    }

    // There should always be a current control method, but just in case, we can log "None" if there
    // isn't one
    Logger.recordOutput(
        "DriveCoordinator/currentControlMethod",
        currentControlMethod == null ? "None" : currentControlMethod.getName());
    Logger.recordOutput("DriveCoordinator/state", driveStateMachine.getCurrentState().getName());
  }

  public void testPeriodic() {
    switch (testModeManager.getTestMode()) {
      case DriveGainsTuning:
        steerGains.ifChanged(hashCode(), drive::setSteerGains);
        driveGains.ifChanged(hashCode(), drive::setDriveGains);
        break;
      default:
        break;
    }
  }
}
