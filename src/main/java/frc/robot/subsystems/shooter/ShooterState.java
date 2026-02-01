package frc.robot.subsystems.shooter;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;

public abstract class ShooterState extends State<ShooterSubsystem> {
  public static class VelocityControlState extends ShooterState {
    @Override
    public void periodic(StateMachine<ShooterSubsystem> stateMachine, ShooterSubsystem shooter) {
      shooter.controlToTargetVelocity();
    }
  }

  public static class TestModeState extends ShooterState {
    @Override
    public void periodic(StateMachine<ShooterSubsystem> stateMachine, ShooterSubsystem shooter) {
      shooter.testPeriodic();
    }
  }
}
