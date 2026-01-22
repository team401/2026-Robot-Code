package frc.robot.subsystems.indexer;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;

public abstract class IndexerState extends State<IndexerSubsystem> {

  public static class IdleState extends IndexerState {
    @Override
    public void periodic(StateMachine<IndexerSubsystem> stateMachine, IndexerSubsystem indexer) {}
  }

  /**
   * TestModeState calls testPeriodic and does nothing else, to allow for the subsystem's
   * testPeriodic method to take action when in test mode without conflict.
   */
  public static class TestModeState extends IndexerState {
    @Override
    public void periodic(StateMachine<IndexerSubsystem> stateMachine, IndexerSubsystem indexer) {
      indexer.testPeriodic();
    }
  }
}
