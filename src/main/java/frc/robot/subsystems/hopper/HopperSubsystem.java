package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFX;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFXSim;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.hopper.HopperState.DejamState;
import frc.robot.subsystems.hopper.HopperState.IdleState;
import frc.robot.subsystems.hopper.HopperState.SpinningState;
import frc.robot.subsystems.hopper.HopperState.TestModeState;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

// I helped copilot autocomplete and chat gpt 5 write this file
public class HopperSubsystem extends MonitoredSubsystem {
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

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

  public HopperSubsystem(MotorIO motor) {
    this.motor = motor;
    stateMachine = new StateMachine<>(this);

    spinningState = (HopperState) stateMachine.registerState(new SpinningState());
    dejamState = (HopperState) stateMachine.registerState(new DejamState());
    idleState = (HopperState) stateMachine.registerState(new IdleState());
    testModeState = (HopperState) stateMachine.registerState(new TestModeState());

    spinningState.whenFinished().transitionTo(idleState);
    spinningState.when(hopper -> hopper.dejamRequired(), "Dejam required").transitionTo(dejamState);
    dejamState.whenFinished().transitionTo(idleState);
    idleState
        .when(hopper -> hopper.isHopperTestMode(), "In hopper test mode")
        .transitionTo(testModeState);
    testModeState
        .when(hopper -> !hopper.isHopperTestMode(), "Not in hopper test mode")
        .transitionTo(idleState);
    stateMachine.setState(spinningState);
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

    hopperExpoKV =
        new LoggedTunableNumber(
            "HopperTunables/hopperExpoKV", JsonConstants.hopperConstants.hopperExpoKV);
    hopperExpoKA =
        new LoggedTunableNumber(
            "HopperTunables/hopperExpoKA", JsonConstants.hopperConstants.hopperExpoKA);

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
    Logger.recordOutput("Hopper/closedLoopReferenceRadians", inputs.closedLoopReference);
    Logger.recordOutput(
        "Hopper/closedLoopReferenceSlopeRadPerSec", inputs.closedLoopReferenceSlope);

    Logger.recordOutput("Hopper/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("Hopper/StateAfter", stateMachine.getCurrentState().getName());
  }

  protected void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case HopperClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sva) -> {
              motor.setGains(
                  pid_sva[0], pid_sva[1], pid_sva[2], pid_sva[3], 0, pid_sva[4], pid_sva[5]);
            },
            hopperKP,
            hopperKI,
            hopperKD,
            hopperKS,
            hopperKV,
            hopperKA);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (maxProfile) -> {
              motor.setProfileConstraints(
                  MotionProfileConfig.immutable(
                      RotationsPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero().div(Seconds.of(1.0)),
                      Volts.of(maxProfile[0]).div(RotationsPerSecond.of(1)),
                      Volts.of(maxProfile[1]).div(RotationsPerSecondPerSecond.of(1))));
            },
            hopperExpoKV,
            hopperExpoKA);

        motor.controlToVelocityUnprofiled(
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

  protected static MotorIO getTalonFXMotorIO() {
    return MotorIOTalonFX.newLeader(MechanismConfig.builder().build(), new TalonFXConfiguration());
  }

  protected static MotorIO getTalonFXMotorSimIO() {
    return MotorIOTalonFXSim.newLeader(
        MechanismConfig.builder().build(), new TalonFXConfiguration());
  }

  public AngularVelocity getHopperVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadiansPerSecond);
  }

  protected void runHopperAtSpeed(AngularVelocity speed) {
    motor.controlToVelocityUnprofiled(speed);
  }

  protected void spin() {
    runHopperAtSpeed(
        RotationsPerSecond.of(JsonConstants.hopperConstants.spinningVoltage.in(Volts)));
  }

  protected void dejam() {
    stopHopper();
    runHopperAtSpeed(
        RotationsPerSecond.of(-JsonConstants.hopperConstants.spinningVoltage.in(Volts)));
  }

  protected void stopHopper() {
    motor.controlToVelocityUnprofiled(RadiansPerSecond.of(0.0));
  }

  protected void coast() {
    motor.controlCoast();
  }

  private boolean dejamRequired() {
    if (!isHopperTestMode()) { // Figure out how to detect jams, current idea is to get the velocity
      // and the current and see if it is stalled unexpectedly like that but I really don't know
      return true;
    }
    return false;
  }

  private boolean isHopperTestMode() {
    return switch (TestModeManager.getTestMode()) {
      case HopperClosedLoopTuning, HopperCurrentTuning, HopperVoltageTuning, HopperPhoenixTuning ->
          true;
      default -> false;
    };
  }
}
