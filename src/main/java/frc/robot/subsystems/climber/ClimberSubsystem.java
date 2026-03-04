package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.climber.ClimberState.HomingWaitForMovementState;
import frc.robot.subsystems.climber.ClimberState.HomingWaitForStoppingState;
import frc.robot.util.StateMachineDump;
import frc.robot.util.TestModeManager;
import java.io.PrintWriter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

// Copilot autocomplete was used to help write this file
public class ClimberSubsystem extends MonitoredSubsystem {
  private enum ClimberAction {
    Wait,
    Home,
    Search,
    Hang,
    Stow
  }

  // Motor IO
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  // State machine
  private final StateMachine<ClimberSubsystem> stateMachine;

  private final ClimberState waitForHomingState;
  private final ClimberState homingWaitForMovementState;
  private final ClimberState homingWaitForStoppingState;
  private final ClimberState stowState; // the armless state
  private final ClimberState searchState; // the big arms boi state
  private final ClimberState hangState; // the t-rex arms state
  private final ClimberState testModeState;

  // State variables
  @AutoLogOutput(key = "Climber/requestedAction")
  private ClimberAction requestedAction = ClimberAction.Wait;

  // Tunables
  // These aren't lazy for now, hopefully we can handle like 20 more logging fields.
  LoggedTunableNumber climberKP;
  LoggedTunableNumber climberKI;
  LoggedTunableNumber climberKD;

  LoggedTunableNumber climberKS;
  LoggedTunableNumber climberKV;
  LoggedTunableNumber climberKA;
  LoggedTunableNumber climberKG;

  LoggedTunableNumber climberExpoKV;
  LoggedTunableNumber climberExpoKA;

  LoggedTunableNumber climberTuningSetpointDegrees;
  LoggedTunableNumber climberTuningAmps;
  LoggedTunableNumber climberTuningVolts;

  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("Climber", TestMode.class);

  public ClimberSubsystem(MotorIO motor) {
    this.motor = motor;
    stateMachine = new StateMachine<>(this);

    waitForHomingState = stateMachine.registerState(new ClimberState.WaitForHomingState());
    homingWaitForMovementState = stateMachine.registerState(new HomingWaitForMovementState());
    homingWaitForStoppingState = stateMachine.registerState(new HomingWaitForStoppingState());
    stowState = stateMachine.registerState(new ClimberState.StowState());
    searchState = stateMachine.registerState(new ClimberState.SearchState());
    hangState = stateMachine.registerState(new ClimberState.HangState());
    testModeState = stateMachine.registerState(new ClimberState.TestModeState());

    waitForHomingState
        .when(
            climber -> DriverStation.isEnabled() && climber.isClimberTestMode(),
            "In climber test mode (must home first)")
        .transitionTo(homingWaitForMovementState);
    waitForHomingState.whenFinished("Should home").transitionTo(homingWaitForMovementState);

    homingWaitForMovementState.whenFinished("Moved").transitionTo(homingWaitForStoppingState);
    homingWaitForMovementState
        .whenTimeout(JsonConstants.climberConstants.homingMaxUnmovingTime)
        .transitionTo(homingWaitForStoppingState);
    homingWaitForStoppingState.whenFinished("Stopped").transitionTo(stowState);
    stowState
        .when(() -> requestedAction == ClimberAction.Search, "Should search")
        .transitionTo(searchState);

    searchState
        .when(() -> requestedAction == ClimberAction.Stow, "Should stow")
        .transitionTo(stowState);
    searchState
        .when(() -> requestedAction == ClimberAction.Hang, "Should hang")
        .transitionTo(hangState);

    hangState
        .when(
            () -> requestedAction == ClimberAction.Search,
            "Requested action is Search (Should declimb)")
        .transitionTo(searchState);

    stowState
        .when(climber -> climber.isClimberTestMode(), "In climber test mode")
        .transitionTo(testModeState);
    testModeState
        .when(climber -> !climber.isClimberTestMode(), "Not in climber test mode")
        .transitionTo(stowState);

    stateMachine.setState(waitForHomingState);
    stateMachine.writeGraphvizFile(new PrintWriter(System.out, true));
    StateMachineDump.write("climber", stateMachine);

    // Initialize tunable numbers for test modes
    climberKP =
        new LoggedTunableNumber(
            "ClimberTunables/climberKP", JsonConstants.climberConstants.climberKP);
    climberKI =
        new LoggedTunableNumber(
            "ClimberTunables/climberKI", JsonConstants.climberConstants.climberKI);
    climberKD =
        new LoggedTunableNumber(
            "ClimberTunables/climberKD", JsonConstants.climberConstants.climberKD);

    climberKS =
        new LoggedTunableNumber(
            "ClimberTunables/climberKS", JsonConstants.climberConstants.climberKS);
    climberKV =
        new LoggedTunableNumber(
            "ClimberTunables/climberKV", JsonConstants.climberConstants.climberKV);
    climberKA =
        new LoggedTunableNumber(
            "ClimberTunables/climberKA", JsonConstants.climberConstants.climberKA);
    climberKG =
        new LoggedTunableNumber(
            "ClimberTunables/climberKG", JsonConstants.climberConstants.climberKG);

    climberExpoKV =
        new LoggedTunableNumber(
            "ClimberTunables/climberExpoKV", JsonConstants.climberConstants.climberExpoKV);
    climberExpoKA =
        new LoggedTunableNumber(
            "ClimberTunables/climberExpoKA", JsonConstants.climberConstants.climberExpoKA);

    climberTuningSetpointDegrees =
        new LoggedTunableNumber("ClimberTunables/climberTuningSetpointDegrees", 0.0);
    climberTuningAmps = new LoggedTunableNumber("ClimberTunables/climberTuningAmps", 0.0);
    climberTuningVolts = new LoggedTunableNumber("ClimberTunables/climberTuningVolts", 0.0);

    AutoLogOutputManager.addObject(this);
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("Climber/inputs", inputs);
    Logger.recordOutput("Climber/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("Climber/StateAfter", stateMachine.getCurrentState().getName());
  }

  protected void testPeriodic() {
    switch (testModeManager.getTestMode()) {
      case ClimberClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sva) -> {
              JsonConstants.climberConstants.climberKP = pid_sva[0];
              JsonConstants.climberConstants.climberKI = pid_sva[1];
              JsonConstants.climberConstants.climberKD = pid_sva[2];
              JsonConstants.climberConstants.climberKS = pid_sva[3];
              JsonConstants.climberConstants.climberKV = pid_sva[4];
              JsonConstants.climberConstants.climberKA = pid_sva[5];
              JsonConstants.climberConstants.climberKG = pid_sva[6];
              motor.setGains(
                  pid_sva[0],
                  pid_sva[1],
                  pid_sva[2],
                  pid_sva[3],
                  pid_sva[6],
                  pid_sva[4],
                  pid_sva[5]);
            },
            climberKP,
            climberKI,
            climberKD,
            climberKS,
            climberKV,
            climberKA,
            climberKG);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (maxProfile) -> {
              JsonConstants.climberConstants.climberExpoKV = maxProfile[0];
              JsonConstants.climberConstants.climberExpoKA = maxProfile[1];
              motor.setProfileConstraints(
                  MotionProfileConfig.immutable(
                      RotationsPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero().div(Seconds.of(1.0)),
                      Volts.of(maxProfile[0]).div(RotationsPerSecond.of(1)),
                      Volts.of(maxProfile[1]).div(RotationsPerSecondPerSecond.of(1))));
            },
            climberExpoKV,
            climberExpoKA);

        motor.controlToPositionExpoProfiled(Degrees.of(climberTuningSetpointDegrees.getAsDouble()));
      }
      case ClimberCurrentTuning -> {
        motor.controlOpenLoopCurrent(Amps.of(climberTuningAmps.getAsDouble()));
      }
      case ClimberVoltageTuning -> {
        motor.controlOpenLoopVoltage(Volts.of(climberTuningVolts.getAsDouble()));
      }
      default -> {}
    }
  }

  protected void coast() {
    motor.controlCoast();
  }

  private boolean isClimberTestMode() {
    return testModeManager.isInTestMode();
  }

  public boolean shouldHome() {
    // TODO: Figure out if homing at the start of teleop is a valid move
    return requestedAction != ClimberAction.Wait;
  }

  public AngularVelocity getClimberVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadiansPerSecond);
  }

  public void setPositionToHomedPosition() {
    motor.setCurrentPosition(JsonConstants.climberConstants.homingAngle);
  }

  protected void applyHomingVoltage() {
    motor.controlOpenLoopVoltage(JsonConstants.climberConstants.homingVoltage);
  }

  public boolean getClimberInUpPosition() {
    return (Math.abs(
            inputs.positionRadians - JsonConstants.climberConstants.upperClimbAngle.in(Degrees))
        < 5);
  }

  public void setToUpperClimbPosition() {
    motor.controlToPositionExpoProfiled(JsonConstants.climberConstants.upperClimbAngle);
  }

  public void setToHangClimbPosition() {
    motor.controlToPositionExpoProfiled(
        JsonConstants.climberConstants.hangClimbAngle); // TODO: Find
  }

  public boolean getClimberInLowerPosition() {
    return (inputs.positionRadians < 5);
  }

  public void setToLowerClimbPosition() {
    motor.controlToPositionExpoProfiled(Degrees.of(0.0));
  }
}
