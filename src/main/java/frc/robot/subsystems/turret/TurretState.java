package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.units.AngularVelocityUnit;
import frc.robot.constants.JsonConstants;

/**
 * The TurretState class contains all states for the TurretSubsystem and defines shared
 * functionality between some states.
 */
public abstract class TurretState extends State<TurretSubsystem> {
  /**
   * Zero the turret's encoder position and finish this state
   *
   * <p>This method exists to avoid duplicated code between multiple states that have to tell the
   * Turret that it is at its homing position and then exit.
   *
   * @param turret The TurretSubsystem to zero
   */
  protected void zeroTurretAndFinish(TurretSubsystem turret) {
    turret.setPositionToHomedPosition();
    finish();
  }

  public static class IdleState extends TurretState {
    @Override
    public void periodic(StateMachine<TurretSubsystem> stateMachine, TurretSubsystem turret) {
      turret.coast();
    }
  }

  /**
   * The HomingWaitForButtonState waits for the homing switch to be pressed and then sets the
   * encoder to its homed position and finishes. The state machine must transition to
   * HomingWaitForMovement state whenever the robot enables, as this state won't do it itself.
   */
  public static class HomingWaitForButtonState extends TurretState {
    @Override
    public void periodic(StateMachine<TurretSubsystem> stateMachine, TurretSubsystem turret) {
      if (turret.getDependencies().isHomingSwitchPressed()) {
        zeroTurretAndFinish(turret);
      }
    }
  }

  /**
   * The HomingWaitForMovementState should be entered if the robot is enabled without the homing
   * switch having been pressed. It applies a gentle voltage output to home the turret and waits for
   * it to begin moving. If it begins moving, it will transition to the HomingWaitForStoppingState.
   * If it doesn't move, it will home by never moving and transition to the IdleState.
   */
  public static class HomingWaitForMovementState extends TurretState {
    @Override
    public void periodic(StateMachine<TurretSubsystem> stateMachine, TurretSubsystem turret) {
      turret.applyHomingVoltage();

      final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
      if (turret.getTurretVelocity().abs(velocityComparisonUnit)
          >= JsonConstants.turretConstants.homingMovementThreshold.in(velocityComparisonUnit)) {
        finish();
      }
    }
  }

  /**
   * The HomingWaitForStoppingState should be entered after the robot is enabled without the homing
   * switch having been pressed and then the turret begins to move. It continues to apply the homing
   * voltage while it waits for the turret to stop moving. Once the turret stops moving, the system
   * is homed and transitions to the IdleState.
   */
  public static class HomingWaitForStoppingState extends TurretState {
    @Override
    public void periodic(StateMachine<TurretSubsystem> stateMachine, TurretSubsystem turret) {
      turret.applyHomingVoltage();

      final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
      if (turret.getTurretVelocity().abs(velocityComparisonUnit)
          < JsonConstants.turretConstants.homingMovementThreshold.in(velocityComparisonUnit)) {
        zeroTurretAndFinish(turret);
      }
    }
  }

  /**
   * TestModeState calls testPeriodic and does nothing else, to allow for the subsystem's
   * testPeriodic method to take action when in test mode without conflict.
   */
  public static class TestModeState extends TurretState {
    @Override
    public void periodic(StateMachine<TurretSubsystem> stateMachine, TurretSubsystem turret) {
      turret.testPeriodic();
    }
  }
}
