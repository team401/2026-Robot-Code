package frc.robot.subsystems.transferRoller;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;

public abstract class TransferRollerState extends State<TransferRollerSubsystem> {

  public static class IdleState extends TransferRollerState {
    @Override
    public void periodic(
        StateMachine<TransferRollerSubsystem> stateMachine, TransferRollerSubsystem intakeRoller) {
      intakeRoller.controlToTargetVelocity();
    }
  }

  /**
   * TestModeState calls testPeriodic and does nothing else, to allow for the subsystem's
   * testPeriodic method to take action when in test mode without conflict.
   */
  public static class TestModeState extends TransferRollerState {
    @Override
    public void periodic(
        StateMachine<TransferRollerSubsystem> stateMachine, TransferRollerSubsystem intakeRoller) {
      intakeRoller.testPeriodic();
    }
  }

  public static class SpinState extends TransferRollerState {
    @Override
    public void periodic(
        StateMachine<TransferRollerSubsystem> stateMachine, TransferRollerSubsystem intakeRoller) {
      intakeRoller.controlToTargetVelocity();
    }
  }

  public static class DeJamState extends TransferRollerState {
    @Override
    public void periodic(
        StateMachine<TransferRollerSubsystem> stateMachine, TransferRollerSubsystem intakeRoller) {
      intakeRoller.controlToTargetVelocity();
    }
  }
}
