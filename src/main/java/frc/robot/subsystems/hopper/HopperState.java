package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.units.AngularVelocityUnit;

// I helped copilot autocomplete write this file
public abstract class HopperState extends State<HopperSubsystem> {
  public static class SpinningState extends HopperState {
    @Override
    public void periodic(StateMachine<HopperSubsystem> stateMachine, HopperSubsystem hopper) {
      hopper.applySpinningVoltage();
      final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
      if (hopper.getHopperVelocity().abs(velocityComparisonUnit)
          >= hopper.hopperTuningSetpointVelocity.getAsDouble()) {
        finish();
      }
    }
  }

  public static class IdleState extends HopperState {
    @Override
    public void periodic(StateMachine<HopperSubsystem> stateMachine, HopperSubsystem hopper) {
      hopper.coast();
      final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
      if (hopper.getHopperVelocity().abs(velocityComparisonUnit)
          < hopper.hopperTuningSetpointVelocity.getAsDouble()) {
        finish();
      }
    }
  }

  public static class DejamState extends HopperState {
    @Override
    public void periodic(
        StateMachine<HopperSubsystem> stateMachine,
        HopperSubsystem hopper) { // TODO: Figure out this logic
      hopper.dejam();
      if (!hopper.dejamRequired()) {
        finish();
      }
    }
  }

  public static class TestModeState extends HopperState {
    @Override
    public void periodic(StateMachine<HopperSubsystem> stateMachine, HopperSubsystem hopper) {
      hopper.testPeriodic();
    }
  }
}
