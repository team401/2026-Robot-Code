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
import frc.robot.subsystems.drive.states.DriveToClimbState;
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
    DriveLinearPath,
    DriveToClimb
  }

  public enum ClimbLocations {
    LeftClimbLocation,
    RightClimbLocation
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
  private DriveToClimbState driveToClimbState;
  static final TestModeManager<TestMode> testModeManager =
      new TestModeManager<>("Drive", TestMode.class);

  public static TestModeManager<TestMode> getTestModeManager() {
    return testModeManager;
  }

  public DriveAction targetAction = DriveAction.DriveWithJoysticks;

  public DriveCoordinator(Drive drive) {
    this.drive = drive;
    // drive.setDriveGains(JsonConstants.driveConstants.driveGains);
    // drive.setSteerGains(JsonConstants.driveConstants.steerGains);

    DISABLED_DRIVE = new DisabledDrive(drive);
    LINEAR_DRIVE = new LinearDrive(drive);
    JOYSTICK_DRIVE =
        DISABLED_DRIVE; // Initialize to disabled drive until joystick drive is initialized

    setControlMethod(DISABLED_DRIVE);

    driveStateMachine = new StateMachine<>(this);

    linearDriveToPoseState = driveStateMachine.registerState(new DrivingLinearPathState());
    driveWithJoysticksState = driveStateMachine.registerState(new DriveWithJoysticksState());
    testModeState = driveStateMachine.registerState(new DriveTestModeState());
    driveToClimbState = driveStateMachine.registerState(new DriveToClimbState());

    // TODO: Decide if we really want to use dedicated transitions for drive or just use setState

    // Test Mode Transitions
    // TODO: Decide if we want to move test mode to not be a state
    testModeState.whenFinished("No Drive Test Mode Active").transitionTo(driveWithJoysticksState);
    driveWithJoysticksState
        .when(testModeManager::isInTestMode, "drive test mode active")
        .transitionTo(testModeState);
    driveToClimbState
      .when(testModeManager::isInTestMode, "drive test mode active")
      .transitionTo(testModeState);
    driveWithJoysticksState
        .when(testModeManager::isInTestMode, "drive test mode active")
        .transitionTo(testModeState);


    driveWithJoysticksState.whenRequestedTransitionTo(linearDriveToPoseState);
    driveWithJoysticksState.whenRequestedTransitionTo(driveToClimbState);
    

    linearDriveToPoseState.whenRequestedTransitionTo(driveWithJoysticksState);
    linearDriveToPoseState.whenRequestedTransitionTo(driveToClimbState);
    linearDriveToPoseState.whenFinished().transitionTo(driveWithJoysticksState);

    driveToClimbState.whenRequestedTransitionTo(driveWithJoysticksState);
    driveToClimbState.whenRequestedTransitionTo(linearDriveToPoseState);
    // TODO: add more climb states for more precise climb line up
    driveToClimbState.whenFinished().transitionTo(driveWithJoysticksState);
    driveStateMachine.setState(driveWithJoysticksState);
    targetAction = DriveAction.DriveWithJoysticks;
  }

  public void initializeJoyStickDriveControl(DriveWithJoysticks command) {
    JOYSTICK_DRIVE = new JoystickDrive(drive, command);
  }

  public void setLinearDriveTarget(Pose2d pose) {
    Logger.recordOutput("DriveCoordinator/linearTarget", pose);
    linearDriveToPoseState.setCommand(
        driveStateMachine, this, new LinearDrive.LinearDriveCommand(pose));
  }

  public void tryToLinearDriveToPose(Pose2d pose) {
    setLinearDriveTarget(pose);
    setDriveTargetAction(DriveAction.DriveLinearPath);
  }

  public void setDriveTargetAction(DriveAction action) {
    targetAction = action;
  }

  public void tryToDriveToClimbLocation(ClimbLocations climbLocation) {
    driveToClimbState.setClimbLocation(climbLocation);
    setDriveTargetAction(DriveAction.DriveToClimb);
  }

  @Override
  public void periodic() {

    switch (targetAction) {
      case DriveWithJoysticks:
        driveStateMachine.requestState(driveWithJoysticksState);
        break;
      case DriveLinearPath:
        driveStateMachine.requestState(linearDriveToPoseState);
        break;
      case DriveToClimb:
        driveStateMachine.requestState(driveToClimbState);
        break;
    }

    driveStateMachine.periodic();

    if (currentControlMethod != null) {
      currentControlMethod.periodic();
    }

    Logger.recordOutput("DriveCoordinator/targetAction", targetAction);
    Logger.recordOutput(
        "DriveCoordinator/Command/name",
        currentControlMethod == null ? "None" : currentControlMethod.getName());
    Logger.recordOutput("DriveCoordinator/state", driveStateMachine.getCurrentState().getName());
  }



  // These exist so that if a seperate drive action was requested, but a state finishes it properly updates
  // the target action

  public void stateFinishAction(DriveAction stateAction) {
    stateNextAction(stateAction, DriveAction.DriveWithJoysticks);
  }

  public void stateNextAction(DriveAction stateAction, DriveAction nextAction) {
    if (targetAction == stateAction) {
      targetAction = nextAction;
    }
  }
}
