package frc.robot.subsystems.climber;

import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;

public class ClimberSubsystem extends MonitoredSubsystem{
    private final MotorIO motor;
    private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
    public static class ClimberDependencies{
        public boolean isHomingSwitchPressed = false;
    }
    private final ClimberDependencies dependencies = new ClimberDependencies();

    private final StateMachine<ClimberSubsystem> stateMachine;

    private final ClimberState homingState;
    @Override
    public ClimberSubsystem(){

    }
    public void monitoredPeriodic() {
        // TODO Auto-generated method stub
    }
}
