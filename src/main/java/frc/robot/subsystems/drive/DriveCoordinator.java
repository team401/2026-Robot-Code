package frc.robot.subsystems.drive;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.states.DriveTestModeState;
import frc.robot.subsystems.drive.states.DriveWithJoysticksState;
import frc.robot.subsystems.drive.states.LinearDriveToPoseState;
import org.littletonrobotics.junction.Logger;

// Copilot used to help write this class

public class DriveCoordinator extends SubsystemBase {

  public enum DriveAction {
    DriveWithJoysticks,
    LinearDriveToPose
  }

  StateMachine<Drive> driveStateMachine;

  private DriveWithJoysticksState driveWithJoysticksState;
  private LinearDriveToPoseState linearDriveToPoseState;
  private DriveTestModeState testModeState;

  private void createDriveStateMachine(Drive drive) {
    driveStateMachine = new StateMachine<>(drive);

    // TODO: Remove casts when CopperCore is updated to latest version

    driveWithJoysticksState =
        (DriveWithJoysticksState) driveStateMachine.registerState(new DriveWithJoysticksState());
    linearDriveToPoseState =
        (LinearDriveToPoseState) driveStateMachine.registerState(new LinearDriveToPoseState());
    testModeState =
        (DriveTestModeState) driveStateMachine.registerState(new DriveTestModeState(drive));

    driveWithJoysticksState.whenRequestedTransitionTo(linearDriveToPoseState);
    driveWithJoysticksState
        .when(_drive -> DriveTestModeState.isDriveTestMode(), "drive test mode active")
        .transitionTo(testModeState);

    linearDriveToPoseState.whenRequestedTransitionTo(driveWithJoysticksState);
    linearDriveToPoseState.whenFinished().transitionTo(driveWithJoysticksState);

    testModeState.whenFinished("No Drive Test Mode Active").transitionTo(driveWithJoysticksState);

    driveStateMachine.setState(driveWithJoysticksState);
  }

  public DriveCoordinator(Drive drive) {
    createDriveStateMachine(drive);
  }

  public void setLinearTargetPose(Pose2d pose) {
    Logger.recordOutput("driveCoordinator/linearTarget", pose);
    linearDriveToPoseState.setTargetPose(pose);
  }

  public void setDriveWithJoysticksCommand(DriveWithJoysticks command) {
    driveWithJoysticksState.setDriveCommand(command);
    testModeState.setDriveCommand(command);
  }

  public void setDriveAction(DriveAction action) {
    switch (action) {
      case DriveWithJoysticks:
        driveStateMachine.requestState(driveWithJoysticksState);
        break;
      case LinearDriveToPose:
        driveStateMachine.requestState(linearDriveToPoseState);
        break;
      // This code should never be called but is a precaution
      default:
        System.err.println("DriveCoordinator: Unknown DriveAction " + action);
        break;
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      driveStateMachine.periodic();
    }
    Logger.recordOutput("driveCoordinator/state", driveStateMachine.getCurrentState().getName());
  }
}
