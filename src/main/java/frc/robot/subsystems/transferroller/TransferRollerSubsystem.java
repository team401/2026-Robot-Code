package frc.robot.subsystems.transferroller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.transferroller.TransferRollerState.DeJamState;
import frc.robot.subsystems.transferroller.TransferRollerState.IdleState;
import frc.robot.subsystems.transferroller.TransferRollerState.SpinState;
import frc.robot.subsystems.transferroller.TransferRollerState.TestModeState;
import frc.robot.util.StateMachineDump;
import frc.robot.util.TestModeManager;
import frc.robot.util.TuningModeHelper;
import frc.robot.util.TuningModeHelper.ControlMode;
import frc.robot.util.TuningModeHelper.MotorTuningMode;
import frc.robot.util.TuningModeHelper.TunableMotor;
import frc.robot.util.TuningModeHelper.TunableMotorConfiguration;
import frc.robot.util.math.Lazy;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TransferRollerSubsystem extends MonitoredSubsystem {
  // Motor and inputs
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  // Desired roller velocity
  private AngularVelocity targetVelocity = RadiansPerSecond.of(0.0);

  private final Debouncer shouldDejamDebouncer =
      new Debouncer(
          JsonConstants.transferRollerConstants.dejamTime.in(Seconds), DebounceType.kRising);

  // State machine and states
  private final StateMachine<TransferRollerSubsystem> stateMachine;

  private final TransferRollerState idleState;
  private final TransferRollerState testModeState;
  private final TransferRollerState spinState;
  private final TransferRollerState deJamState;

  // Tuning Helper and Test Mode Manager
  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("TransferRoller", TestMode.class);

  Lazy<TuningModeHelper<TestMode>> tuningModeHelper;

  public TransferRollerSubsystem(MotorIO motor) {
    this.motor = motor;

    stateMachine = new StateMachine<>(this);
    idleState = stateMachine.registerState(new IdleState());
    testModeState = stateMachine.registerState(new TestModeState());
    spinState = stateMachine.registerState(new SpinState());
    deJamState = stateMachine.registerState(new DeJamState());

    idleState
        .when(
            transferRoller -> transferRoller.isTransferRollerTestMode(),
            "In transfer roller test mode")
        .transitionTo(testModeState);

    idleState
        .when(transferRoller -> transferRoller.shouldSpin(), "Postive target velocity requested")
        .transitionTo(spinState);

    idleState
        .when(transferRoller -> transferRoller.shouldDeJam(), "Negative target velocity requested")
        .transitionTo(deJamState);

    testModeState
        .when(
            transferRoller -> !transferRoller.isTransferRollerTestMode(),
            "Not in transfer roller test mode")
        .transitionTo(idleState);

    spinState
        .when(
            transferRoller -> !transferRoller.shouldSpin(),
            "Non-positive target velocity requested")
        .transitionTo(idleState);

    spinState
        .when(transferRoller -> transferRoller.shouldDeJam(), "Should dejam")
        .transitionTo(deJamState);

    deJamState.whenTimeout(JsonConstants.transferRollerConstants.dejamTime).transitionTo(spinState);

    stateMachine.setState(idleState);
    StateMachineDump.write("transferroller", stateMachine);

    // Initialize tuning mode helper
    Supplier<TunableMotor> createTunableMotor =
        () ->
            TunableMotorConfiguration.defaultConfiguration()
                .withVelocityTuning()
                .profiled()
                .withDefaultMotionProfileConfig(
                    JsonConstants.transferRollerConstants.transferRollerMotionProfileConfig)
                .withDefaultPIDGains(JsonConstants.transferRollerConstants.transferRollerGains)
                .onPIDGainsChanged(
                    newGains ->
                        JsonConstants.transferRollerConstants.transferRollerGains = newGains)
                .onMotionProfileConfigChanged(
                    newProfile ->
                        JsonConstants.transferRollerConstants.transferRollerMotionProfileConfig =
                            newProfile)
                .withTunableAngularVelocityUnit(RPM)
                .build("TransferRoller/MotorTuning", motor);

    tuningModeHelper =
        new Lazy<>(
            () ->
                new TuningModeHelper<TestMode>(TestMode.class)
                    .addMotorTuningModes(
                        createTunableMotor.get(),
                        MotorTuningMode.of(
                            TestMode.TransferRollerClosedLoopTuning, ControlMode.CLOSED_LOOP),
                        MotorTuningMode.of(
                            TestMode.TransferRollerCurrentTuning, ControlMode.OPEN_LOOP_CURRENT),
                        MotorTuningMode.of(
                            TestMode.TransferRollerVoltageTuning, ControlMode.OPEN_LOOP_VOLTAGE),
                        MotorTuningMode.of(
                            TestMode.TransferRollerPhoenixTuning, ControlMode.PHOENIX_TUNING),
                        MotorTuningMode.of(TestMode.None, ControlMode.NONE)));
  }

  @Override
  public void monitoredPeriodic() {
    long startTimeUs = RobotController.getFPGATime();

    motor.updateInputs(inputs);
    Logger.processInputs("TransferRoller/inputs", inputs);

    Logger.recordOutput("TransferRoller/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();

    long endTimeUs = RobotController.getFPGATime();
    if (JsonConstants.featureFlags.logPeriodicTiming) {
      Logger.recordOutput("PeriodicTime/TransferRollerMs", (endTimeUs - startTimeUs) / 1000.0);
    }
  }

  protected void testPeriodic() {
    tuningModeHelper.get().testPeriodic(testModeManager.getTestMode());
  }

  /**
   * Check TestModeManager for whether or not the currently selected test mode requires the transfer
   * roller to switch to its tuning state.
   *
   * @return True if the robot is enabled in test mode with a transfer roller test mode selected,
   *     false if a non-transfer roller test mode is selected, the test mode doesn't require the
   *     transfer roller to enter TestModeState, the robot isn't enabled in test mode, or
   *     TestModeManager hasn't been initialized.
   */
  private boolean isTransferRollerTestMode() {
    return testModeManager.isInTestMode();
  }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadiansPerSecond);
  }

  public void controlToTargetVelocity() {
    motor.controlToVelocityProfiled(targetVelocity);
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
  }

  public boolean shouldSpin() {
    return targetVelocity.in(Units.RadiansPerSecond) > 0;
  }

  public boolean shouldDeJam() {
    return shouldDejamDebouncer.calculate(
        inputs.velocityRadiansPerSecond < targetVelocity.in(RadiansPerSecond) / 10
            && inputs.statorCurrentAmps
                > JsonConstants.transferRollerConstants.transferRollerStatorCurrentLimit.in(Amps)
                    / 2);
  }

  void coast() {
    motor.controlCoast();
  }
}
