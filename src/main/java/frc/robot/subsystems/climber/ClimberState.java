package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.units.AngularVelocityUnit;
import frc.robot.constants.JsonConstants;

// Copilot autocomplete was used to help write this file
public abstract class ClimberState extends State<ClimberSubsystem> {
  public static class WaitForHomingState extends ClimberState {
    @Override
    public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
      if (climber.shouldHome()) {
        finish();
      }
    }
  }

  public static class HomingWaitForMovementState extends ClimberState {
    @Override
    public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
      climber.applyHomingVoltage();
      final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
      if (climber.getClimberVelocity().abs(velocityComparisonUnit)
          >= JsonConstants.climberConstants.homingMovementThreshold.in(velocityComparisonUnit)) {
        finish();
      }
    }
  }

  public static class HomingWaitForStoppingState extends ClimberState {
    @Override
    public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
      climber.applyHomingVoltage();
      final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
      if (climber.getClimberVelocity().abs(velocityComparisonUnit)
          < JsonConstants.climberConstants.homingMovementThreshold.in(velocityComparisonUnit)) {
        climber.setPositionToHomedPosition();
        finish();
      }
    }
  }

  public static class IdleState extends ClimberState {
    @Override
    public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
      climber.coast();
    }
  }

  public static class TestModeState extends ClimberState {
    @Override
    public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
      climber.testPeriodic();
    }
  }

  public static class SearchingState extends ClimberState {
    @Override
    public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
      climber.setToUpperClimbPosition();
    }
  }
}
