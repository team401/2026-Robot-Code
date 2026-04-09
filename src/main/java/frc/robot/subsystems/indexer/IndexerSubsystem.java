package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.indexer.IndexerState.IdleState;
import frc.robot.subsystems.indexer.IndexerState.ShootingState;
import frc.robot.subsystems.indexer.IndexerState.TestModeState;
import frc.robot.subsystems.indexer.IndexerState.WarmupState;
import frc.robot.util.StateMachineDump;
import frc.robot.util.TestModeManager;
import frc.robot.util.TotalCurrentCalculator;
import frc.robot.util.TuningModeHelper;
import frc.robot.util.TuningModeHelper.ControlMode;
import frc.robot.util.TuningModeHelper.MotorTuningMode;
import frc.robot.util.TuningModeHelper.TunableMotor;
import frc.robot.util.TuningModeHelper.TunableMotorConfiguration;
import frc.robot.util.math.Lazy;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
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

  // Tuning Helper and Test Mode Manager
  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("Indexer", TestMode.class);

  Lazy<TuningModeHelper<TestMode>> tuningModeHelper;

  private final Debouncer isAtGoalVelocityDebouncer =
      new Debouncer(
          JsonConstants.indexerConstants.atSetpointDebounceTime.in(Seconds), DebounceType.kFalling);

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
    StateMachineDump.write("indexer", stateMachine);

    // Initialize tuning mode helper
    Supplier<TunableMotor> createTunableMotor =
        () ->
            TunableMotorConfiguration.defaultConfiguration()
                .withVoltageClosedLoopTuning()
                .profiled()
                .withDefaultMotionProfileConfig(
                    JsonConstants.indexerConstants.indexerMotionProfileConfig)
                .withDefaultPIDGains(JsonConstants.indexerConstants.indexerGains)
                .onPIDGainsChanged(
                    newGains -> JsonConstants.indexerConstants.indexerGains = newGains)
                .onMotionProfileConfigChanged(
                    newProfile ->
                        JsonConstants.indexerConstants.indexerMotionProfileConfig = newProfile)
                .withTunableAngularVelocityUnit(RPM)
                .build("Indexer/MotorTuning", motor);

    tuningModeHelper =
        new Lazy<>(
            () ->
                new TuningModeHelper<TestMode>(TestMode.class)
                    .addMotorTuningModes(
                        createTunableMotor.get(),
                        MotorTuningMode.of(
                            TestMode.IndexerClosedLoopTuning, ControlMode.CLOSED_LOOP),
                        MotorTuningMode.of(
                            TestMode.IndexerCurrentTuning, ControlMode.OPEN_LOOP_CURRENT),
                        MotorTuningMode.of(
                            TestMode.IndexerVoltageTuning, ControlMode.OPEN_LOOP_VOLTAGE),
                        MotorTuningMode.of(
                            TestMode.IndexerPhoenixTuning, ControlMode.PHOENIX_TUNING),
                        MotorTuningMode.of(TestMode.None, ControlMode.NONE)));
  }

  @Override
  public void monitoredPeriodic() {
    long startTimeUs = RobotController.getFPGATime();

    motor.updateInputs(inputs);
    Logger.processInputs("Indexer/inputs", inputs);

    TotalCurrentCalculator.reportCurrent(hashCode(), inputs.supplyCurrentAmps);

    Logger.recordOutput("Indexer/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();

    long endTimeUs = RobotController.getFPGATime();
    if (JsonConstants.featureFlags.logPeriodicTiming) {
      Logger.recordOutput("PeriodicTime/indexerMs", (endTimeUs - startTimeUs) / 1000.0);
    }
  }

  protected void testPeriodic() {
    tuningModeHelper.get().testPeriodic(testModeManager.getTestMode());
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
    if (targetVelocity.abs(RPM) < 5.0) {
      motor.controlBrake();
    } else {
      motor.controlToVelocityProfiledVoltage(targetVelocity);
    }
  }

  // If the target velocity is not zero, the indexer should start the shooting process
  public boolean shouldShoot() {
    return !targetVelocity.equals(RadiansPerSecond.of(0));
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
  }

  @AutoLogOutput(key = "Indexer/readyToShoot")
  public boolean readyToShoot() {
    return isAtGoalVelocityDebouncer.calculate(
        getVelocity()
            .isNear(
                targetVelocity,
                JsonConstants.indexerConstants.indexerMaximumRelativeVelocityError));
  }
}
