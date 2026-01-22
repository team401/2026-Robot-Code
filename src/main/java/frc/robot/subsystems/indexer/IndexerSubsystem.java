package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.indexer.IndexerState.IdleState;
import frc.robot.subsystems.indexer.IndexerState.TestModeState;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends MonitoredSubsystem {
  // Motor and inputs
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  // State machine and states
  private final StateMachine<IndexerSubsystem> stateMachine;

  private final IndexerState idleState;
  private final IndexerState testModeState;

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

  LoggedTunableNumber indexerTuningAmps;
  LoggedTunableNumber indexerTuningVolts;
  LoggedTunableNumber indexerTuningSetpointVelocity;

  public IndexerSubsystem(MotorIO motor) {
    this.motor = motor;

    // Define state machine transitions, register states
    stateMachine = new StateMachine<>(this);
    idleState = (IndexerState) stateMachine.registerState(new IdleState());
    testModeState = (IndexerState) stateMachine.registerState(new TestModeState());

    idleState
        .when(indexer -> indexer.isIndexerTestMode(), "In indexer test mode")
        .transitionTo(testModeState);

    testModeState
        .when(indexer -> !indexer.isIndexerTestMode(), "Not in indexer test mode")
        .transitionTo(idleState);

    stateMachine.setState(testModeState);

    // Initialize tunable numbers for test modes
    indexerKP =
        new LoggedTunableNumber(
            "IndexerTunables/indexerKP", JsonConstants.indexerConstants.indexerKP);
    indexerKI =
        new LoggedTunableNumber(
            "IndexerTunables/indexerKI", JsonConstants.indexerConstants.indexerKI);
    indexerKD =
        new LoggedTunableNumber(
            "IndexerTunables/indexerKD", JsonConstants.indexerConstants.indexerKD);

    indexerKS =
        new LoggedTunableNumber(
            "IndexerTunables/indexerKS", JsonConstants.indexerConstants.indexerKS);
    indexerKV =
        new LoggedTunableNumber(
            "IndexerTunables/indexerKV", JsonConstants.indexerConstants.indexerKV);
    indexerKA =
        new LoggedTunableNumber(
            "IndexerTunables/indexerKA", JsonConstants.indexerConstants.indexerKA);

    indexerExpoKV =
        new LoggedTunableNumber(
            "IndexerTunables/indexerExpoKV", JsonConstants.indexerConstants.indexerExpoKV);
    indexerExpoKA =
        new LoggedTunableNumber(
            "IndexerTunables/indexerExpoKA", JsonConstants.indexerConstants.indexerExpoKA);

    indexerTuningSetpointVelocity =
        new LoggedTunableNumber("IndexerTunables/indexerTuningSetpointVelocity", 0.0);
    indexerTuningAmps = new LoggedTunableNumber("IndexerTunables/indexerTuningAmps", 0.0);
    indexerTuningVolts = new LoggedTunableNumber("IndexerTunables/indexerTuningVolts", 0.0);
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("Indexer/inputs", inputs);
    Logger.recordOutput("Indexer/velocityRadiansPerSecond", inputs.velocityRadiansPerSecond);

    Logger.recordOutput("Indexer/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("Indexer/StateAfter", stateMachine.getCurrentState().getName());
  }

  protected void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case IndexerClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sva) -> {
              motor.setGains(
                  pid_sva[0], pid_sva[1], pid_sva[2], pid_sva[3], 0, pid_sva[4], pid_sva[5]);
            },
            indexerKP,
            indexerKI,
            indexerKD,
            indexerKS,
            indexerKV,
            indexerKA);

        motor.controlToVelocityUnprofiled(
            RadiansPerSecond.of(indexerTuningSetpointVelocity.getAsDouble()));
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

  /**
   * Check TestModeManager for whether or not the currently selected test mode requires the indexer
   * to switch to its tuning state.
   *
   * @return True if the robot is enabled in test mode with a indexer test mode selected, false if a
   *     non-indexer test mode is selected, the test mode doesn't require the indexer to enter
   *     TestModeState, the robot isn't enabled in test mode, or TestModeManager hasn't been
   *     initialized.
   */
  private boolean isIndexerTestMode() {
    return switch (TestModeManager.getTestMode()) {
      case IndexerClosedLoopTuning,
          IndexerCurrentTuning,
          IndexerVoltageTuning,
          IndexerPhoenixTuning ->
          true;
      default -> false;
    };
  }
}
