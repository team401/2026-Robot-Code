package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.hopper.HopperState.DejamState;
import frc.robot.subsystems.hopper.HopperState.IdleState;
import frc.robot.subsystems.hopper.HopperState.SpinningState;
import frc.robot.subsystems.hopper.HopperState.TestModeState;
import frc.robot.util.TestModeManager;
import frc.robot.util.TuningModeHelper;
import frc.robot.util.TuningModeHelper.ControlMode;
import frc.robot.util.TuningModeHelper.MotorTuningMode;
import frc.robot.util.TuningModeHelper.TunableMotor;
import frc.robot.util.TuningModeHelper.TunableMotorConfiguration;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

// I helped copilot autocomplete and chat gpt 5 write this file
public class HopperSubsystem extends MonitoredSubsystem {
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  private AngularVelocity targetVelocity = RadiansPerSecond.of(0.0);

  private final StateMachine<HopperSubsystem> stateMachine;
  private final HopperState spinningState;
  private final HopperState dejamState;
  private final HopperState idleState;
  private final HopperState testModeState;

  TuningModeHelper<TestMode> tuningModeHelper;

  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("Hopper", TestMode.class);

  public HopperSubsystem(MotorIO motor) {
    this.motor = motor;
    stateMachine = new StateMachine<>(this);

    spinningState = stateMachine.registerState(new SpinningState());
    dejamState = stateMachine.registerState(new DejamState());
    idleState = stateMachine.registerState(new IdleState());
    testModeState = stateMachine.registerState(new TestModeState());

    spinningState.when(hopper -> hopper.shouldIdle(), "Should idle").transitionTo(idleState);
    spinningState
        .when(hopper -> hopper.isHopperTestMode(), "In hopper test mode")
        .transitionTo(testModeState);
    spinningState.when(hopper -> hopper.dejamRequired(), "Dejam required").transitionTo(dejamState);
    dejamState
        .when(hopper -> !hopper.dejamRequired(), "Dejam not required")
        .transitionTo(spinningState);
    idleState.when(hopper -> !hopper.shouldIdle(), "Should spin").transitionTo(spinningState);
    idleState
        .when(hopper -> hopper.isHopperTestMode(), "In hopper test mode")
        .transitionTo(testModeState);
    testModeState
        .when(hopper -> !hopper.isHopperTestMode(), "Not in hopper test mode")
        .transitionTo(idleState);
    stateMachine.setState(idleState);

    // Initialize tuning mode helper
    TunableMotor tunableMotor =
        TunableMotorConfiguration.defaultConfiguration()
            .withVelocityTuning()
            .profiled()
            .withDefaultMotionProfileConfig(
                JsonConstants.hopperConstants.hopperMotionProfileConfig)
            .withDefaultPIDGains(JsonConstants.hopperConstants.hopperGains)
            .onPIDGainsChanged(newGains -> JsonConstants.hopperConstants.hopperGains = newGains)
            .onMotionProfileConfigChanged(
                newProfile ->
                    JsonConstants.hopperConstants.hopperMotionProfileConfig = newProfile)
            .withLoggingAngularVelocityUnit(RPM)
            .build("Hopper/MotorTuning", motor);

    tuningModeHelper = new TuningModeHelper<TestMode>( TestMode.class)
            .addMotorTuningModes(tunableMotor, 
              MotorTuningMode.of(TestMode.HopperClosedLoopTuning, ControlMode.CLOSED_LOOP),
              MotorTuningMode.of(TestMode.HopperCurrentTuning, ControlMode.OPEN_LOOP_CURRENT),
              MotorTuningMode.of(TestMode.HopperVoltageTuning, ControlMode.OPEN_LOOP_VOLTAGE),
              MotorTuningMode.of(TestMode.HopperPhoenixTuning, ControlMode.PHOENIX_TUNING),
              MotorTuningMode.of(TestMode.None, ControlMode.NONE)
            );

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("Hopper/inputs", inputs);
    Logger.recordOutput("Hopper/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("Hopper/StateAfter", stateMachine.getCurrentState().getName());
  }

  protected void testPeriodic() {
    tuningModeHelper.testPeriodic(testModeManager.getTestMode());
  }

  private boolean isHopperTestMode() {
    return testModeManager.isInTestMode();
  }

  private boolean shouldIdle() {
    return false; // TODO: ask if the hopper should be idling at all
  }

  public AngularVelocity getHopperVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadiansPerSecond);
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
  }

  public void setToTargetVelocity() {
    motor.controlToVelocityProfiled(targetVelocity);
  }

  protected void applyVoltage(Voltage volts) {
    motor.controlOpenLoopVoltage(volts);
  }

  protected void dejam() {
    stopHopper();
    applyVoltage(JsonConstants.hopperConstants.dejamVoltage);
  }

  protected void
      stopHopper() { // This method might actually be useless, i don't think it stops the hopper
    applyVoltage(Volts.of(0.0));
  }

  protected void coast() {
    motor.controlCoast();
  }

  public boolean dejamRequired() {
    final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;
    boolean notSpinning =
        getHopperVelocity().abs(velocityComparisonUnit)
            < JsonConstants.hopperConstants.spinningMovementThreshold.in(velocityComparisonUnit);
    boolean highCurrent =
        inputs.statorCurrentAmps
            > JsonConstants.hopperConstants.dejamCurrentThreshold.in(Amps); // Figure out this logic
    if (notSpinning && highCurrent) {
      return true;
    }
    return false;
  }
}
