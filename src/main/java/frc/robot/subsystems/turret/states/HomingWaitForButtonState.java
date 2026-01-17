package frc.robot.subsystems.turret.states;

import coppercore.controls.state_machine.StateMachine;
import frc.robot.subsystems.turret.BaseTurretState;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * The HomingWaitForButtonState waits for the homing switch to be pressed and then sets the encoder
 * to its homed position and finishes. The state machine must transition to HomingWaitForMovement
 * state whenever the robot enables, as this state won't do it itself.
 */
public class HomingWaitForButtonState extends BaseTurretState {
  @Override
  public void periodic(StateMachine<TurretSubsystem> stateMachine, TurretSubsystem turret) {
    if (turret.getDependenciesObject().isHomingSwitchPressed) {
      setTurretPositionToHomedPosition(turret);
      finish();
    }
  }
}
