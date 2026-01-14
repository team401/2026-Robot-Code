package frc.robot.subsystems.turret.states;

import coppercore.controls.state_machine.StateMachine;
import frc.robot.subsystems.turret.BaseTurretState;
import frc.robot.subsystems.turret.TurretSubsystem;

public class IdleState extends BaseTurretState {
  @Override
  public void periodic(StateMachine<TurretSubsystem> stateMachine, TurretSubsystem turret) {
    coastTurret(turret);
  }
}
