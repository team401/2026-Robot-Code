package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.units.AngularVelocityUnit;
import frc.robot.constants.JsonConstants;

/**
 * The HoodState class contains all states for the HoodSubsystem and defines shared functionality
 * between the states.
 */
public abstract class HoodState extends State<HoodSubsystem> {
  /**
   * Home the hood (set its current position to the "homed" position) and finish the current state
   *
   * <p>This method is not referred to as "zeroing" the hood because the homing position likely
   * won't be at 0 degrees.
   *
   * @param hood The HoodSubsystem to home
   */
  protected void homeHoodAndFinish(HoodSubsystem hood) {
    hood.homePositionReached();
    finish();
  }

  /**
   * The HomingWaitForButtonState waits for the homing switch to be pressed and then sets the hood
   * motor's encoder position to its homed position and finishes. The state machine must transition
   * to HomingWaitForMovement state whenever the robot enables.
   */
  public static class HomingWaitForButtonState extends HoodState {
    @Override
    public void periodic(StateMachine<HoodSubsystem> stateMachine, HoodSubsystem hood) {
      if (hood.isHomingSwitchPressed()) {
        homeHoodAndFinish(hood);
      }
    }
  }

  /**
   * The HomingWaitForMovementState should be entered if the robot is enabled without the homing
   * switch having been pressed. It applies a gentle voltage down into the hardstop and waits for it
   * to begin moving. If it begins moving, it will transition to HomingWaitForStoppingState and
   * waits for it to stop. If it doesn't move after a certain timeout, it will home by never moving
   * and transition to the IdleState, by transitioning to HomingWaitForStoppingState, which will
   * instantly home and finish due to the fact that the hood isn't moving.
   */
  public static class HomingWaitForMovementState extends HoodState {
    @Override
    public void periodic(StateMachine<HoodSubsystem> stateMachine, HoodSubsystem hood) {
      hood.applyHomingVoltage();

      final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
      if (hood.getVelocity().abs(velocityComparisonUnit)
          >= JsonConstants.turretConstants.homingMovementThreshold.in(velocityComparisonUnit)) {
        finish();
      }
    }
  }

  /**
   * The HomingWaitForStoppingState should be entered after the HomingWaitForMovement state either
   * detects movement or times out. It will continue to apply the homing voltage and, as soon as the
   * hood is not moving, home the hood and finish.
   */
  public static class HomingWaitForStoppingState extends HoodState {
    @Override
    public void periodic(StateMachine<HoodSubsystem> stateMachine, HoodSubsystem hood) {
      hood.applyHomingVoltage();

      final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
      if (hood.getVelocity().abs(velocityComparisonUnit)
          < JsonConstants.turretConstants.homingMovementThreshold.in(velocityComparisonUnit)) {
        homeHoodAndFinish(hood);
      }
    }
  }

  public static class IdleState extends HoodState {
    @Override
    public void periodic(StateMachine<HoodSubsystem> stateMachine, HoodSubsystem hood) {
      hood.coast();
    }
  }

  /**
   * The TargetPitchState continually commands the hood to target its goal pitch, as commanded by
   * the coordination layer
   */
  public static class TargetPitchState extends HoodState {
    @Override
    public void periodic(StateMachine<HoodSubsystem> stateMachine, HoodSubsystem hood) {
      hood.controlToGoalPitch();
    }
  }

  public static class TestModeState extends HoodState {
    @Override
    public void periodic(StateMachine<HoodSubsystem> stateMachine, HoodSubsystem hood) {
      hood.testPeriodic();
    }
  }
}
