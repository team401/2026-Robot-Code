package frc.robot.subsystems.turret.states;

import coppercore.controls.state_machine.StateMachine;
import frc.robot.subsystems.turret.BaseTurretState;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * A state that does nothing, to allow for the subsystem's testsPeriodic to take action when in test
 * mode.
 */
public class TestModeState extends BaseTurretState {
  @Override
  public void periodic(StateMachine<TurretSubsystem> stateMachine, TurretSubsystem turret) {
    runTestPeriodic(turret);
  }
}
