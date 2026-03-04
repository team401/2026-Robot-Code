package frc.robot.subsystems.transferRoller;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.transferRoller.TransferRollerState.DeJamState;
import frc.robot.subsystems.transferRoller.TransferRollerState.IdleState;
import frc.robot.subsystems.transferRoller.TransferRollerState.SpinState;
import frc.robot.subsystems.transferRoller.TransferRollerState.TestModeState;
import frc.robot.util.StateMachineDump;
import frc.robot.util.TestModeManager;
import frc.robot.util.TuningModeHelper;
import frc.robot.util.TuningModeHelper.ControlMode;
import frc.robot.util.TuningModeHelper.MotorTuningMode;
import frc.robot.util.TuningModeHelper.TunableMotor;
import frc.robot.util.TuningModeHelper.TunableMotorConfiguration;
import org.littletonrobotics.junction.Logger;

public class TransferRollerSubsystem extends MonitoredSubsystem {
  // Motor and inputs
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  // Desired roller velocity
  private AngularVelocity targetVelocity = RadiansPerSecond.of(0.0);

  // State machine and states
  private final StateMachine<TransferRollerSubsystem> stateMachine;

  private final TransferRollerState idleState;
  private final TransferRollerState testModeState;
  private final TransferRollerState spinState;
  private final TransferRollerState deJamState;

  // Tuning Helper and Test Mode Manager
  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("TransferRoller", TestMode.class);

  TuningModeHelper<TestMode> tuningModeHelper;

  public TransferRollerSubsystem(MotorIO motor) {
    this.motor = motor;

    // TODO: Define state machine transitions, register states
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

    deJamState
        .when(
            transferRoller -> !transferRoller.shouldDeJam(),
            "Non-negative target velocity requested")
        .transitionTo(idleState);

    stateMachine.setState(idleState);
    StateMachineDump.write("transferroller", stateMachine);

    // Initialize tuning mode helper
    TunableMotor tunableMotor =
        TunableMotorConfiguration.defaultConfiguration()
            .withVelocityTuning()
            .profiled()
            .withDefaultMotionProfileConfig(
                JsonConstants.transferRollerConstants.transferRollerMotionProfileConfig)
            .withDefaultPIDGains(JsonConstants.transferRollerConstants.transferRollerGains)
            .onPIDGainsChanged(
                newGains -> JsonConstants.transferRollerConstants.transferRollerGains = newGains)
            .onMotionProfileConfigChanged(
                newProfile ->
                    JsonConstants.transferRollerConstants.transferRollerMotionProfileConfig =
                        newProfile)
            .withTunableAngularVelocityUnit(RPM)
            .build("TransferRoller/MotorTuning", motor);

    tuningModeHelper =
        new TuningModeHelper<TestMode>(TestMode.class)
            .addMotorTuningModes(
                tunableMotor,
                MotorTuningMode.of(
                    TestMode.TransferRollerClosedLoopTuning, ControlMode.CLOSED_LOOP),
                MotorTuningMode.of(
                    TestMode.TransferRollerCurrentTuning, ControlMode.OPEN_LOOP_CURRENT),
                MotorTuningMode.of(
                    TestMode.TransferRollerVoltageTuning, ControlMode.OPEN_LOOP_VOLTAGE),
                MotorTuningMode.of(
                    TestMode.TransferRollerPhoenixTuning, ControlMode.PHOENIX_TUNING),
                MotorTuningMode.of(TestMode.None, ControlMode.NONE));
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("TransferRoller/inputs", inputs);

    Logger.recordOutput("TransferRoller/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("TransferRoller/StateAfter", stateMachine.getCurrentState().getName());
  }

  protected void testPeriodic() {
    tuningModeHelper.testPeriodic(testModeManager.getTestMode());
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
    return targetVelocity.in(Units.RadiansPerSecond) < 0;
  }
}
