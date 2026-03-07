package frc.robot.subsystems.hopper;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;

// I helped copilot autocomplete write this file
public abstract class HopperState extends State<HopperSubsystem> {
  public static class SpinningState extends HopperState {
    @Override
    public void periodic(StateMachine<HopperSubsystem> stateMachine, HopperSubsystem hopper) {
      hopper.setToTargetVelocity();
    }
  }

  public static class IdleState extends HopperState {
    @Override
    public void periodic(StateMachine<HopperSubsystem> stateMachine, HopperSubsystem hopper) {
      hopper.coast();
    }
  }

  /** Wiggles the hopper forward in idle when the shouldWiggleInIdle constant is set to true */
  public static class WiggleState extends HopperState {
    @Override
    public void periodic(StateMachine<HopperSubsystem> stateMachine, HopperSubsystem hopper) {
      hopper.coast();
    }
  }

  public static class DejamState extends HopperState {
    @Override
    public void periodic(
        StateMachine<HopperSubsystem> stateMachine,
        HopperSubsystem hopper) { // TODO: Figure out this logic
      hopper.dejam();
    }
  }

  public static class TestModeState extends HopperState {
    @Override
    public void periodic(StateMachine<HopperSubsystem> stateMachine, HopperSubsystem hopper) {
      hopper.testPeriodic();
    }
  }
}
