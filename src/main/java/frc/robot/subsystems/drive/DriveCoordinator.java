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
import frc.robot.subsystems.drive.states.DriveTestModeState;
import frc.robot.subsystems.drive.states.DriveWithJoysticksState;
import frc.robot.subsystems.drive.states.DrivingLinearPathState;
import frc.robot.util.TestModeManager;

import org.littletonrobotics.junction.Logger;

// Copilot used to help write this class

// States currently are more so what is drive trying to do
// And Control Methods are more so how drive is trying to do it

public class DriveCoordinator extends SubsystemBase {

  public enum DriveAction {
    DriveWithJoysticks,
    DriveLinearPath
  }

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

  StateMachine<DriveCoordinator> driveStateMachine;
  public Drive drive;

  private DriveWithJoysticksState driveWithJoysticksState;
  private DrivingLinearPathState linearDriveToPoseState;
  private DriveTestModeState testModeState;
  static final TestModeManager<TestMode> testModeManager =
      new TestModeManager<>("Drive", TestMode.class);

  public static TestModeManager<TestMode> getTestModeManager() {
    return testModeManager;
  }

  public DriveAction targetAction = DriveAction.DriveWithJoysticks;
  /** Current Action being null means its outside of normal operation such as Test Mode */
  public DriveAction currentAction = DriveAction.DriveWithJoysticks;

  public DriveCoordinator(Drive drive) {
    this.drive = drive;
    drive.setDriveGains(JsonConstants.driveConstants.driveGains);
    drive.setSteerGains(JsonConstants.driveConstants.steerGains);

    DISABLED_DRIVE = new DisabledDrive(drive);
    LINEAR_DRIVE = new LinearDrive(drive);
    JOYSTICK_DRIVE = DISABLED_DRIVE; // Initialize to disabled drive until joystick drive is initialized

    setControlMethod(DISABLED_DRIVE);

    driveStateMachine = new StateMachine<>(this);

    linearDriveToPoseState = driveStateMachine.registerState(new DrivingLinearPathState());
    driveWithJoysticksState = driveStateMachine.registerState(new DriveWithJoysticksState());
    testModeState = driveStateMachine.registerState(new DriveTestModeState());

    driveWithJoysticksState.whenRequestedTransitionTo(linearDriveToPoseState);
    driveWithJoysticksState
        .when(testModeManager::isInTestMode, "drive test mode active")
        .transitionTo(testModeState);

    linearDriveToPoseState.whenRequestedTransitionTo(driveWithJoysticksState);
    linearDriveToPoseState.whenFinished().transitionTo(driveWithJoysticksState);

    testModeState.whenFinished("No Drive Test Mode Active").transitionTo(driveWithJoysticksState);

    driveStateMachine.setState(driveWithJoysticksState);
    targetAction = DriveAction.DriveWithJoysticks;
    currentAction = DriveAction.DriveWithJoysticks;
  }

  public void initializeJoyStickDriveControl(DriveWithJoysticks command) {
    JOYSTICK_DRIVE = new JoystickDrive(drive, command);
  }

  public void setLinearDriveTarget(Pose2d pose) {
    Logger.recordOutput("driveCoordinator/linearTarget", pose);
    linearDriveToPoseState.setCommand(driveStateMachine, this, new LinearDrive.LinearDriveCommand(pose));
  }

  public void tryToLinearDriveToPose(Pose2d pose) {
    setLinearDriveTarget(pose);
    setDriveTargetAction(DriveAction.DriveLinearPath);
  }

  public void setDriveTargetAction(DriveAction action) {
    targetAction = action;
  }

  @Override
  public void periodic() {

    if (currentAction != null && currentAction != targetAction) {
      switch (targetAction) {
        case DriveWithJoysticks:
          driveStateMachine.requestState(driveWithJoysticksState);
          break;
        case DriveLinearPath:
          driveStateMachine.requestState(linearDriveToPoseState);
          break;
      }
    }

    driveStateMachine.periodic();

    if (currentControlMethod != null) {
      currentControlMethod.periodic();
    }

    Logger.recordOutput("driveCoordinator/state", driveStateMachine.getCurrentState().getName());
  }
}
