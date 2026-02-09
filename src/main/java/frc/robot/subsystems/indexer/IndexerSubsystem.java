package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.indexer.IndexerState.IdleState;
import frc.robot.subsystems.indexer.IndexerState.ShootingState;
import frc.robot.subsystems.indexer.IndexerState.TestModeState;
import frc.robot.subsystems.indexer.IndexerState.WarmupState;
import frc.robot.util.TestModeManager;
import frc.robot.util.TuningModeHelper;
import frc.robot.util.TuningModeHelper.ControlMode;
import frc.robot.util.TuningModeHelper.MotorTuningMode;
import frc.robot.util.TuningModeHelper.TunableMotor;
import frc.robot.util.TuningModeHelper.TunableMotorConfiguration;

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

  // Tuning Helper and Test Mode Manager
  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("Indexer", TestMode.class);

  TuningModeHelper<TestMode> tuningModeHelper;

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

    // Initialize tuning mode helper
    TunableMotor tunableMotor =
        TunableMotorConfiguration.defaultConfiguration()
            .withVelocityTuning()
            .profiled()
            .withDefaultMotionProfileConfig(
                JsonConstants.indexerConstants.indexerMotionProfileConfig)
            .withDefaultPIDGains(JsonConstants.indexerConstants.indexerGains)
            .onPIDGainsChanged(newGains -> JsonConstants.indexerConstants.indexerGains = newGains)
            .onMotionProfileConfigChanged(
                newProfile ->
                    JsonConstants.indexerConstants.indexerMotionProfileConfig = newProfile)
            .build("Indexer", motor);
    
    tuningModeHelper =
        new TuningModeHelper<TestMode>(TestMode.class)
            .addMotorTuningModes(tunableMotor, 
                MotorTuningMode.of(TestMode.IndexerClosedLoopTuning, ControlMode.CLOSED_LOOP),
                MotorTuningMode.of(TestMode.IndexerCurrentTuning, ControlMode.OPEN_LOOP_CURRENT),
                MotorTuningMode.of(TestMode.IndexerVoltageTuning, ControlMode.OPEN_LOOP_VOLTAGE),
                MotorTuningMode.of(TestMode.IndexerPhoenixTuning, ControlMode.PHOENIX_TUNING),
                MotorTuningMode.of(TestMode.None, ControlMode.NONE)
            );

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
    tuningModeHelper.runTestMode(testModeManager.getTestMode());
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
