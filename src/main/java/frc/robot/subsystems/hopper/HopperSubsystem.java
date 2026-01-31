package frc.robot.subsystems.hopper;

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
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.hopper.HopperState.DejamState;
import frc.robot.subsystems.hopper.HopperState.IdleState;
import frc.robot.subsystems.hopper.HopperState.SpinningState;
import frc.robot.subsystems.hopper.HopperState.TestModeState;
import frc.robot.util.TestModeManager;
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

  LoggedTunableNumber hopperKP;
  LoggedTunableNumber hopperKI;
  LoggedTunableNumber hopperKD;

  LoggedTunableNumber hopperKS;
  LoggedTunableNumber hopperKV;
  LoggedTunableNumber hopperKA;
  LoggedTunableNumber hopperKG;

  LoggedTunableNumber hopperExpoKV;
  LoggedTunableNumber hopperExpoKA;

  LoggedTunableNumber hopperTuningSetpointVelocity;
  LoggedTunableNumber hopperTuningAmps;
  LoggedTunableNumber hopperTuningVolts;

  LoggedTunableNumber hopperMaxAccelerationRotationsPerSecondSquared;
  LoggedTunableNumber hopperMaxJerkRotationsPerSecondCubed;

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
    hopperKP =
        new LoggedTunableNumber("HopperTunables/hopperKP", JsonConstants.hopperConstants.hopperKP);
    hopperKI =
        new LoggedTunableNumber("HopperTunables/hopperKI", JsonConstants.hopperConstants.hopperKI);
    hopperKD =
        new LoggedTunableNumber("HopperTunables/hopperKD", JsonConstants.hopperConstants.hopperKD);

    hopperKS =
        new LoggedTunableNumber("HopperTunables/hopperKS", JsonConstants.hopperConstants.hopperKS);
    hopperKV =
        new LoggedTunableNumber("HopperTunables/hopperKV", JsonConstants.hopperConstants.hopperKV);
    hopperKA =
        new LoggedTunableNumber("HopperTunables/hopperKA", JsonConstants.hopperConstants.hopperKA);

    hopperMaxAccelerationRotationsPerSecondSquared =
        new LoggedTunableNumber(
            "HopperTunables/hopperMaxAccelerationRotationsPerSecond",
            JsonConstants.hopperConstants.hopperMaxAccelerationRotationsPerSecondSquared);
    hopperMaxJerkRotationsPerSecondCubed =
        new LoggedTunableNumber(
            "HopperTunables/hopperMaxJerkRotationsPerSecond",
            JsonConstants.hopperConstants.hopperMaxJerkRotationsPerSecondCubed);

    hopperTuningSetpointVelocity =
        new LoggedTunableNumber("HopperTunables/hopperTuningSetpointVelocity", 0.0);
    hopperTuningAmps = new LoggedTunableNumber("HopperTunables/hopperTuningAmps", 0.0);
    hopperTuningVolts = new LoggedTunableNumber("HopperTunables/hopperTuningVolts", 0.0);

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
    switch (testModeManager.getTestMode()) {
      case HopperClosedLoopTuning -> {
        // if the user changes any of the gains, update the motor gains
        // and the hopperConstants
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sva) -> {
              JsonConstants.hopperConstants.hopperKP = pid_sva[0];
              JsonConstants.hopperConstants.hopperKI = pid_sva[1];
              JsonConstants.hopperConstants.hopperKD = pid_sva[2];
              JsonConstants.hopperConstants.hopperKS = pid_sva[3];
              JsonConstants.hopperConstants.hopperKV = pid_sva[4];
              JsonConstants.hopperConstants.hopperKA = pid_sva[5];
              motor.setGains(
                  pid_sva[0], pid_sva[1], pid_sva[2], pid_sva[3], 0, pid_sva[4], pid_sva[5]);
            },
            hopperKP,
            hopperKI,
            hopperKD,
            hopperKS,
            hopperKV,
            hopperKA);

        // if the user changes acceleration or jerk, update the motor profile
        // and the hopperConstants
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (acc_jerk) -> {
              JsonConstants.hopperConstants.hopperMaxAccelerationRotationsPerSecondSquared =
                  acc_jerk[0];
              JsonConstants.hopperConstants.hopperMaxJerkRotationsPerSecondCubed = acc_jerk[1];
              motor.setProfileConstraints(
                  MotionProfileConfig.immutable(
                      RotationsPerSecond.zero(),
                      RotationsPerSecondPerSecond.of(acc_jerk[0]),
                      RotationsPerSecondPerSecond.of(acc_jerk[1]).div(Seconds.of(1.0)),
                      Volts.zero().div(RotationsPerSecond.of(1)),
                      Volts.zero().div(RotationsPerSecondPerSecond.of(1))));
            },
            hopperMaxAccelerationRotationsPerSecondSquared,
            hopperMaxJerkRotationsPerSecondCubed);

        motor.controlToVelocityProfiled(
            RadiansPerSecond.of(hopperTuningSetpointVelocity.getAsDouble()));
      }
      case HopperCurrentTuning -> {
        motor.controlOpenLoopCurrent(Amps.of(hopperTuningAmps.getAsDouble()));
      }
      case HopperVoltageTuning -> {
        motor.controlOpenLoopVoltage(Volts.of(hopperTuningVolts.getAsDouble()));
      }
      default -> {}
    }
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
