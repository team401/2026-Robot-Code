package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.indexer.IndexerState.IdleState;
import frc.robot.subsystems.indexer.IndexerState.ShootingState;
import frc.robot.subsystems.indexer.IndexerState.TestModeState;
import frc.robot.subsystems.indexer.IndexerState.WarmupState;
import frc.robot.util.TestModeManager;
import java.io.PrintWriter;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends MonitoredSubsystem {
  // Motor and inputs
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  // Desired shooting velocity
  private AngularVelocity targetVelocity = RadiansPerSecond.of(0.0);

  // State machine and states
  private final StateMachine<IndexerSubsystem> stateMachine;

  private final IndexerState idleState;
  private final IndexerState testModeState;
  private final IndexerState warmupState;
  private final IndexerState shootingState;

  // Tunable numbers
  LoggedTunableNumber indexerKP;
  LoggedTunableNumber indexerKI;
  LoggedTunableNumber indexerKD;

  LoggedTunableNumber indexerKS;
  LoggedTunableNumber indexerKV;
  LoggedTunableNumber indexerKA;

  LoggedTunableNumber indexerMaxAccelerationRotationsPerSecondSquared;
  LoggedTunableNumber indexerMaxJerkRotationsPerSecondCubed;

  LoggedTunableNumber indexerTuningAmps;
  LoggedTunableNumber indexerTuningVolts;
  LoggedTunableNumber indexerTuningSetpointVelocityRPM;

  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("Indexer", TestMode.class);

  public IndexerSubsystem(MotorIO motor) {
    this.motor = motor;

    // Define state machine transitions, register states
    stateMachine = new StateMachine<>(this);
    idleState = stateMachine.registerState(new IdleState());
    testModeState = stateMachine.registerState(new TestModeState());
    warmupState = stateMachine.registerState(new WarmupState());
    shootingState = stateMachine.registerState(new ShootingState());

    idleState
        .when(indexer -> indexer.isIndexerTestMode(), "In indexer test mode")
        .transitionTo(testModeState);

    idleState
        .when(indexer -> indexer.shouldShoot(), "Nonzero targetVelocity requested")
        .transitionTo(warmupState);

    testModeState
        .when(indexer -> !indexer.isIndexerTestMode(), "Not in indexer test mode")
        .transitionTo(idleState);

    warmupState
        .when(indexer -> indexer.readyToShoot(), "Indexer velocity at target")
        .transitionTo(shootingState);

    warmupState.when(indexer -> !indexer.shouldShoot(), "Warmup aborted").transitionTo(idleState);

    shootingState
        .when(indexer -> !indexer.shouldShoot(), "Shooting aborted")
        .transitionTo(idleState);

    shootingState
        .when(
            indexer -> !indexer.readyToShoot() && indexer.shouldShoot(), "Changing shooting speed")
        .transitionTo(warmupState);

    stateMachine.setState(idleState);
    stateMachine.writeGraphvizFile(new PrintWriter(System.out, true));

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

    indexerMaxAccelerationRotationsPerSecondSquared =
        new LoggedTunableNumber(
            "IndexerTunables/indexerMaxAccelerationRotationsPerSecond",
            JsonConstants.indexerConstants.indexerMaxAccelerationRotationsPerSecondSquared);
    indexerMaxJerkRotationsPerSecondCubed =
        new LoggedTunableNumber(
            "IndexerTunables/indexerMaxJerkRotationsPerSecond",
            JsonConstants.indexerConstants.indexerMaxJerkRotationsPerSecondCubed);

    indexerTuningSetpointVelocityRPM =
        new LoggedTunableNumber("IndexerTunables/indexerTuningSetpointVelocityRPM", 0.0);
    indexerTuningAmps = new LoggedTunableNumber("IndexerTunables/indexerTuningAmps", 0.0);
    indexerTuningVolts = new LoggedTunableNumber("IndexerTunables/indexerTuningVolts", 0.0);
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("Indexer/inputs", inputs);

    Logger.recordOutput("Indexer/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("Indexer/StateAfter", stateMachine.getCurrentState().getName());
  }

  protected void testPeriodic() {
    switch (testModeManager.getTestMode()) {
      case IndexerClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sva) -> {
              JsonConstants.indexerConstants.indexerKP = pid_sva[0];
              JsonConstants.indexerConstants.indexerKI = pid_sva[1];
              JsonConstants.indexerConstants.indexerKD = pid_sva[2];
              JsonConstants.indexerConstants.indexerKS = pid_sva[3];
              JsonConstants.indexerConstants.indexerKV = pid_sva[4];
              JsonConstants.indexerConstants.indexerKA = pid_sva[5];
              motor.setGains(
                  pid_sva[0], pid_sva[1], pid_sva[2], pid_sva[3], 0, pid_sva[4], pid_sva[5]);
            },
            indexerKP,
            indexerKI,
            indexerKD,
            indexerKS,
            indexerKV,
            indexerKA);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (acc_jerk) -> {
              JsonConstants.indexerConstants.indexerMaxAccelerationRotationsPerSecondSquared =
                  acc_jerk[0];
              JsonConstants.indexerConstants.indexerMaxJerkRotationsPerSecondCubed = acc_jerk[1];
              motor.setProfileConstraints(
                  MotionProfileConfig.immutable(
                      RotationsPerSecond.zero(),
                      RotationsPerSecondPerSecond.of(acc_jerk[0]),
                      RotationsPerSecondPerSecond.of(acc_jerk[1]).div(Seconds.of(1.0)),
                      Volts.zero().div(RotationsPerSecond.of(1)),
                      Volts.zero().div(RotationsPerSecondPerSecond.of(1))));
            },
            indexerMaxAccelerationRotationsPerSecondSquared,
            indexerMaxJerkRotationsPerSecondCubed);

        motor.controlToVelocityProfiled(
            RadiansPerSecond.of(indexerTuningSetpointVelocityRPM.getAsDouble()));
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
    return testModeManager.isInTestMode();
  }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadiansPerSecond);
  }

  public void setToTargetVelocity() {
    motor.controlToVelocityProfiled(targetVelocity);
  }

  // If the target velocity is not zero, the indexer should start the shooting process
  public boolean shouldShoot() {
    return !targetVelocity.equals(RadiansPerSecond.of(0));
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
  }

  public boolean readyToShoot() {
    return getVelocity()
        .isNear(targetVelocity, JsonConstants.indexerConstants.indexerMaximumRelativeVelocityError);
  }
}
