package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
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
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.climber.ClimberState.HomingWaitForMovementState;
import frc.robot.subsystems.climber.ClimberState.HomingWaitForStoppingState;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.StateMachineDump;
import frc.robot.util.TestModeManager;
import frc.robot.util.TotalCurrentCalculator;
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
  private final ClimberState stowState; // Stowed state (arms retracted)
  private final ClimberState searchState; // Search/extend state (looking for rung)
  private final ClimberState hangState; // Hang state (arms engaged for hanging) (t-rex arms)
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

  LoggedTunableMeasure<MutVoltage, Voltage, VoltageUnit> hangVoltage =
      new LoggedTunableMeasure<>(
          "ClimberTunables/hangVoltage",
          JsonConstants.climberConstants.hangClimbVoltage.mutableCopy(),
          Volts,
          true);

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
        .transitionTo(homingWaitForMovementState);
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
    long startTimeUs = RobotController.getFPGATime();

    motor.updateInputs(inputs);
    Logger.processInputs("Climber/inputs", inputs);

    TotalCurrentCalculator.reportCurrent(hashCode(), inputs.supplyCurrentAmps);

    Logger.recordOutput("Climber/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();

    long endTimeUs = RobotController.getFPGATime();
    Logger.recordOutput("PeriodicTime/climberMs", (endTimeUs - startTimeUs) / 1000.0);
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
    return requestedAction != ClimberAction.Wait
        && stateMachine.getCurrentState() == waitForHomingState;
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

  public void setToUpperClimbPosition() {
    motor.controlToPositionExpoProfiled(JsonConstants.climberConstants.upperClimbAngle);
  }

  public void setToStowPosition() {
    motor.controlToPositionExpoProfiled(JsonConstants.climberConstants.stowAngle);
  }

  public void setToHangClimbPosition() {
    motor.controlToPositionExpoProfiled(
        JsonConstants.climberConstants.hangClimbAngle); // TODO: Find
  }

  protected void applyHangVoltage() {
    // TODO: Switch back to the constant after we find a good value for this
    motor.controlOpenLoopVoltage(JsonConstants.climberConstants.hangClimbVoltage);
  }

  public boolean isStowed() {
    return (inputs.positionRadians <= JsonConstants.climberConstants.maxStowedAngle.in(Radians));
  }

  protected boolean isWithinStowCoastThreshold() {
    return Math.abs(inputs.positionRadians - JsonConstants.climberConstants.stowAngle.in(Radians))
        < JsonConstants.climberConstants.stowCoastMargin.in(Radians);
  }

  public void setToLowerClimbPosition() {
    motor.controlToPositionExpoProfiled(Degrees.of(0.0));
  }

  /**
   * Stows the climber if it has been homed, or waits to home if it hasn't been homed.
   *
   * <p>This method should only be called by the CoordinationLayer
   */
  public void stayStowed() {
    requestedAction =
        switch (requestedAction) {
          case Wait -> ClimberAction.Wait;
          case Hang -> ClimberAction.Hang;
          default -> ClimberAction.Stow;
        };
  }

  public boolean isHanging() {
    return stateMachine.getCurrentState() == hangState || requestedAction == ClimberAction.Hang;
  }

  /**
   * Sets the climber's requested action to be stowed.
   *
   * <p>This method should only be called by the CoordinationLayer
   */
  public void stow() {
    requestedAction = ClimberAction.Stow;
  }

  /**
   * Sets the climber's requested action to be searching. This won't guarantee that it immediately
   * searches, it may have to home beforehand.
   *
   * <p>This method should only be called by the CoordinationLayer
   */
  public void search() {
    requestedAction = ClimberAction.Search;
  }

  /**
   * Sets the climber's requested action to be hanging. This means that the climber will soon be
   * commanded to go downward with climbing gains.
   */
  public void hang() {
    requestedAction = ClimberAction.Hang;
  }

  /**
   * Checks whether or not the climber is below its stow threshold position or it has not been
   * homed.
   *
   * <p>This operates under the assumption that the climber is always stowed before the match, so it
   * is stowed prior to being homed.
   *
   * @return {@code false} if the climber has been homed and is above its stow threshold, {@code
   *     true} otherwise.
   */
  public boolean isStowedOrHasntBeenHomed() {
    return stateMachine.getCurrentState() == waitForHomingState || isStowed();
  }

  public boolean isAtSearchPosition() {
    Angle position = Radians.of(inputs.positionRadians);
    return position.isNear(
        JsonConstants.climberConstants.upperClimbAngle,
        JsonConstants.climberConstants.climbSearchAngleMargin);
  }

  /**
   * Checks whether the climber is above its hanging setpoint. If this method returns true and the
   * climber is trying to hang, it should drive downward.
   *
   * @return {@code true} if the climber is above its hanging setpoint, {@code false} otherwise.
   */
  public boolean isAboveHangPosition() {
    return inputs.positionRadians > JsonConstants.climberConstants.hangClimbAngle.in(Radians);
  }

  public Distance getHeightMeters() {
    return Meters.of(inputs.positionRadians * 0.0145);
  }
}
