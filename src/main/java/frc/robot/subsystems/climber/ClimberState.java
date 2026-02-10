package frc.robot.subsystems.climber;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;

// Copilot autocomplete was used to help write this file 
public abstract class ClimberState extends State<ClimberSubsystem>{
    public static class HomingWaitForButtonState extends ClimberState{
        @Override
        public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
            climber.setToHomedPosition();
            if(climber.dependencies.isHomingSwitchPressed()){
                finish();
            }
        }
    }
    public static class IdleState extends ClimberState{
        @Override
        public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
            climber.coast();
        }
    }
    public static class TestModeState extends ClimberState{
        @Override
        public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
            climber.testPeriodic();
        }
    }
    public static class ClimbLevelOneState extends ClimberState{
        @Override
        public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
            climber.setToUpperClimbPosition();
            climber.setToHomedPosition();
        }
    }
    public static class ClimbUpOneLevel extends ClimberState{
        @Override
        public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
            climber.setToUpperClimbPosition();
            //Move servo out
            climber.setToHomedPosition();
        }
    }
    public static class DeClimbState extends ClimberState{
        @Override
        public void periodic(StateMachine<ClimberSubsystem> stateMachine, ClimberSubsystem climber) {
            climber.setToUpperClimbPosition();
        }
    }
    
}
