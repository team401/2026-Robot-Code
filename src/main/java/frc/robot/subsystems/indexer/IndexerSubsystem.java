package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import frc.robot.TestModeManager;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends MonitoredSubsystem {
  // Motor and inputs
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  // Tunable numbers
  LoggedTunableNumber indexerKP;
  LoggedTunableNumber indexerKI;
  LoggedTunableNumber indexerKD;

  LoggedTunableNumber indexerKS;
  LoggedTunableNumber indexerKV;
  LoggedTunableNumber indexerKA;
  LoggedTunableNumber indexerKG;

  LoggedTunableNumber indexerExpoKV;
  LoggedTunableNumber indexerExpoKA;

  LoggedTunableNumber indexerTuningSetpointVelocity;
  LoggedTunableNumber indexerTuningAmps;
  LoggedTunableNumber indexerTuningVolts;

  public IndexerSubsystem(MotorIO motor) {
    this.motor = motor;
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("Indexer/inputs", inputs);
    Logger.recordOutput("Indexer/velocityRadiansPerSecond", inputs.velocityRadiansPerSecond);

    // Implement these once state machine is added
    /*
    Logger.recordOutput("Indexer/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("Indexer/StateAfter", stateMachine.getCurrentState().getName());
    */
  }

  protected void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case IndexerClosedLoopTuning -> {
        // TODO: figure this out
      }
      case IndexerCurrentTuning -> {
        motor.controlOpenLoopCurrent(Amps.of(indexerTuningAmps.getAsDouble()));
      }
      case IndexerVoltageTuning -> {
        motor.controlOpenLoopVoltage(Volts.of(indexerTuningVolts.getAsDouble()));
      }
      default -> {}
    }
  }
}
