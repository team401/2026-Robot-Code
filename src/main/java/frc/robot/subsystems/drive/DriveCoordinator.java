package frc.robot.subsystems.drive;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.states.DriveWithJoysticksState;
import frc.robot.subsystems.drive.states.LinearDriveToPoseState;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// Copilot used to help write this class

public class DriveCoordinator extends SubsystemBase {

  enum DriveAction {
    DriveWithJoysticks,
    LinearDriveToPose,
    Idle
  }

  StateMachine<Drive> driveStateMachine;

  private DriveWithJoysticksState driveWithJoysticksState;
  private LinearDriveToPoseState linearDriveToPoseState;

  private void createDriveStateMachine(Drive drive) {
    driveStateMachine = new StateMachine<>(drive);

    driveWithJoysticksState = new DriveWithJoysticksState();
    linearDriveToPoseState = new LinearDriveToPoseState();

    List.of(driveWithJoysticksState, linearDriveToPoseState)
        .forEach(driveStateMachine::registerState);

    driveWithJoysticksState.whenRequestedTransitionTo(linearDriveToPoseState);

    linearDriveToPoseState.whenRequestedTransitionTo(driveWithJoysticksState);
    linearDriveToPoseState.whenFinished().transitionTo(driveWithJoysticksState);

    driveStateMachine.setState(driveWithJoysticksState);
  }

  public DriveCoordinator(Drive drive) {
    createDriveStateMachine(drive);
  }

  public void setLinearTargetPose(Pose2d pose) {
    linearDriveToPoseState.setTargetPose(pose);
  }

  public void setDriveWithJoysticksCommand(DriveWithJoysticks command) {
    if (driveStateMachine.getCurrentState().equals(driveWithJoysticksState)) {
      DriveWithJoysticks currentCommand = driveWithJoysticksState.getDriveCommand();
      if (currentCommand != null) {
        CommandScheduler.getInstance().cancel(currentCommand);
      }
      driveWithJoysticksState.setDriveCommand(command);
      CommandScheduler.getInstance().schedule(command);
      return;
    }
    driveWithJoysticksState.setDriveCommand(command);
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
