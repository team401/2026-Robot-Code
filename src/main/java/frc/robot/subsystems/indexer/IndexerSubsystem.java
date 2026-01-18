package frc.robot.subsystems.indexer;

import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends MonitoredSubsystem {
  // Motor and inputs
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  public IndexerSubsystem(MotorIO motor) {
    this.motor = motor;
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("Indexer/inputs", inputs);

    // Implement these once state machine is added
    /*
    Logger.recordOutput("Indexer/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("Indexer/StateAfter", stateMachine.getCurrentState().getName());
    */
  }
}
