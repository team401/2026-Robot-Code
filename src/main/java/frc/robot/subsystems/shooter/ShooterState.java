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

  public static class CoastState extends ShooterState {
    @Override
    public void periodic(StateMachine<ShooterSubsystem> stateMachine, ShooterSubsystem shooter) {
      // Coast the shooter rather than controlling it to 0 with closed loop to avoid exploding the
      // belts unnecessarily
      shooter.coast();
    }
  }

  public static class TestModeState extends ShooterState {
    @Override
    public void periodic(StateMachine<ShooterSubsystem> stateMachine, ShooterSubsystem shooter) {
      shooter.testPeriodic();
    }
  }
}
