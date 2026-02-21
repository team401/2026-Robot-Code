package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitorWithAlert.MonitorWithAlertBuilder;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.UnitUtils;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.DependencyOrderedExecutor;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.hood.HoodState.HomingWaitForButtonState;
import frc.robot.subsystems.hood.HoodState.HomingWaitForMovementState;
import frc.robot.subsystems.hood.HoodState.HomingWaitForStoppingState;
import frc.robot.subsystems.hood.HoodState.IdleState;
import frc.robot.subsystems.hood.HoodState.TargetAngleState;
import frc.robot.subsystems.hood.HoodState.TargetPitchState;
import frc.robot.subsystems.hood.HoodState.TestModeState;
import frc.robot.util.TestModeManager;
import java.io.PrintWriter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * The HoodSubsystem class defines the Hood subsystem, which controls the hardware for the hood on
 * the aimer of our shooter superstructure. It uses simple closed loop control to accomplish its
 * tasks.
 */
public class HoodSubsystem extends MonitoredSubsystem {
  private enum HoodAction {
    /** Coasts the hood and waits for input */
    Idle,
    /** Targets a certain pitch, commanded by the supervisor layer */
    TargetPitch,
    /** Targets a certain angle, commanded by the supervisor layer */
    TargetAngle,
  }

  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  public static final ActionKey UPDATE_INPUTS = new ActionKey("HoodSubsystem::updateInputs");

  /**
   * Whether or not the homing switch is currently pressed. This value is updated by the
   * CoordinationLayer via the setIsHomingSwitchPressed method.
   */
  private boolean isHomingSwitchPressed = false;

  // State machine and states
  private final StateMachine<HoodSubsystem> stateMachine;

  private final HoodState homingWaitForButtonState;
  private final HoodState homingWaitForMovementState;
  private final HoodState homingWaitForStoppingState;
  private final HoodState idleState;
  private final HoodState targetPitchState;
  private final HoodState targetAngleState;
  private final HoodState testModeState;

  // Test mode
  TestModeManager<TestMode> testModeManager = new TestModeManager<TestMode>("Hood", TestMode.class);

  // Tunable numbers
  private final LoggedTunableNumber hoodKP;
  private final LoggedTunableNumber hoodKI;
  private final LoggedTunableNumber hoodKD;

  private final LoggedTunableNumber hoodKS;
  private final LoggedTunableNumber hoodKV;
  private final LoggedTunableNumber hoodKA;
  private final LoggedTunableNumber hoodKG;

  private final LoggedTunableNumber hoodExpoKV;
  private final LoggedTunableNumber hoodExpoKA;

  private final LoggedTunableNumber hoodTuningSetpointDegrees;
  private final LoggedTunableNumber hoodTuningAmps;
  private final LoggedTunableNumber hoodTuningVolts;

  // State variables
  /**
   * The last action requested of the Hood. This is the action that the hood should target, but only
   * if the state machine allows. For example, it should only track a pitch after homing.
   */
  @AutoLogOutput(key = "Hood/action")
  private HoodAction requestedAction = HoodAction.Idle;

  private MutAngle goalPitch =
      JsonConstants.hoodConstants
          .minHoodAngle
          .plus(JsonConstants.hoodConstants.mechanismAngleToExitAngle)
          .mutableCopy();

  private MutAngle goalAngle = JsonConstants.hoodConstants.minHoodAngle.mutableCopy();

  public HoodSubsystem(DependencyOrderedExecutor dependencyOrderedExecutor, MotorIO motor) {
    this.motor = motor;

    addMonitor(
        new MonitorWithAlertBuilder()
            .withName("HoodMotorDisconnected")
            .withAlertText("Hood motor disconnected")
            .withAlertType(AlertType.kError)
            .withTimeToFault(JsonConstants.hoodConstants.disconnectedDebounceTimeSeconds)
            .withLoggingEnabled(true)
            .withStickyness(false)
            .withIsStateValidSupplier(() -> inputs.connected)
            .withFaultCallback(() -> {})
            .build());

    // Define state machine and transitions
    stateMachine = new StateMachine<>(this);

    homingWaitForButtonState = stateMachine.registerState(new HomingWaitForButtonState());
    homingWaitForMovementState = stateMachine.registerState(new HomingWaitForMovementState());
    homingWaitForStoppingState = stateMachine.registerState(new HomingWaitForStoppingState());
    idleState = stateMachine.registerState(new IdleState());
    targetPitchState = stateMachine.registerState(new TargetPitchState());
    targetAngleState = stateMachine.registerState(new TargetAngleState());
    testModeState = stateMachine.registerState(new TestModeState());

    homingWaitForButtonState
        .when(hood -> hood.isHomingSwitchPressed(), "Homing switch is pressed")
        .transitionTo(idleState);
    homingWaitForButtonState
        .when(() -> DriverStation.isEnabled(), "Robot is enabled")
        .transitionTo(homingWaitForMovementState);

    homingWaitForMovementState
        .when((hood) -> hood.isMoving(), "Is moving")
        .transitionTo(homingWaitForStoppingState);
    homingWaitForMovementState
        .whenTimeout(JsonConstants.hoodConstants.homingMaxUnmovingTime)
        .transitionTo(homingWaitForStoppingState);

    homingWaitForStoppingState
        .when(hood -> !hood.isMoving(), "Is not moving")
        .transitionTo(idleState);

    idleState.when(hood -> hood.isHoodTestMode(), "Is hood test mode").transitionTo(testModeState);
    idleState
        .when(hood -> hood.requestedAction == HoodAction.TargetPitch, "Action == TargetPitch")
        .transitionTo(targetPitchState);
    idleState
        .when(hood -> hood.requestedAction == HoodAction.TargetAngle, "Action == TargetAngle")
        .transitionTo(targetAngleState);

    targetPitchState
        .when(hood -> hood.requestedAction != HoodAction.TargetPitch, "Action != TargetPitch")
        .transitionTo(idleState);

    targetAngleState
        .when(hood -> hood.requestedAction != HoodAction.TargetAngle, "Action != TargetAngle")
        .transitionTo(idleState);

    testModeState
        .when(hood -> !hood.isHoodTestMode(), "Isn't hood test mode")
        .transitionTo(idleState);

    System.out.println("Hood state machine graph:");
    stateMachine.writeGraphvizFile(new PrintWriter(System.out, true));
    stateMachine.setState(homingWaitForButtonState);

    // Initialize tunable numbers for test modes
    hoodKP = new LoggedTunableNumber("HoodTunables/HoodKP", JsonConstants.hoodConstants.hoodKP);
    hoodKI = new LoggedTunableNumber("HoodTunables/HoodKI", JsonConstants.hoodConstants.hoodKI);
    hoodKD = new LoggedTunableNumber("HoodTunables/HoodKD", JsonConstants.hoodConstants.hoodKD);

    hoodKS = new LoggedTunableNumber("HoodTunables/HoodKS", JsonConstants.hoodConstants.hoodKS);
    hoodKG = new LoggedTunableNumber("HoodTunables/HoodKG", JsonConstants.hoodConstants.hoodKG);
    hoodKV = new LoggedTunableNumber("HoodTunables/HoodKV", JsonConstants.hoodConstants.hoodKV);
    hoodKA = new LoggedTunableNumber("HoodTunables/HoodKA", JsonConstants.hoodConstants.hoodKA);

    hoodExpoKV =
        new LoggedTunableNumber("HoodTunables/HoodExpoKV", JsonConstants.hoodConstants.hoodExpoKV);
    hoodExpoKA =
        new LoggedTunableNumber("HoodTunables/HoodExpoKA", JsonConstants.hoodConstants.hoodExpoKA);

    hoodTuningSetpointDegrees =
        new LoggedTunableNumber(
            "HoodTunables/HoodTuningSetpointDegrees",
            JsonConstants.hoodConstants.minHoodAngle.in(Degrees));
    hoodTuningAmps = new LoggedTunableNumber("HoodTunables/HoodAmps", 0.0);
    hoodTuningVolts = new LoggedTunableNumber("HoodTunables/HoodVolts", 0.0);

    dependencyOrderedExecutor.registerAction(UPDATE_INPUTS, this::updateInputs);

    AutoLogOutputManager.addObject(this);
  }

  private void updateInputs() {
    motor.updateInputs(inputs);
    Logger.processInputs("Hood/inputs", inputs);

    // Log values with units so that AdvantageScope can understand them correctly
    Logger.recordOutput("Hood/closedLoopReferenceRadians", inputs.closedLoopReference);
    Logger.recordOutput("Hood/closedLoopReferenceSlopeRadPerSec", inputs.closedLoopReferenceSlope);

    // For some reason, AutoLogOutput doesn't log the unit correctly, so we have to log it here.
    Logger.recordOutput("Hood/exitAngleRadians", getCurrentExitAngle().in(Radians));

    Logger.recordOutput(
        "Hood/bottomAngleRadians",
        inputs.positionRadians - JsonConstants.hoodConstants.minHoodAngle.in(Radians));
  }

  @Override
  public void monitoredPeriodic() {
    Logger.recordOutput("Hood/state", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("Hood/stateAfter", stateMachine.getCurrentState().getName());
  }

  /**
   * Polls for test-mode specific actions (such as updating gains from network tables)
   *
   * <p>This method must be called by the test mode state, as it does not run automatically
   */
  protected void testPeriodic() {
    switch (testModeManager.getTestMode()) {
      case HoodClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sgva) -> {
              JsonConstants.hoodConstants.hoodKP = pid_sgva[0];
              JsonConstants.hoodConstants.hoodKI = pid_sgva[1];
              JsonConstants.hoodConstants.hoodKD = pid_sgva[2];
              JsonConstants.hoodConstants.hoodKS = pid_sgva[3];
              JsonConstants.hoodConstants.hoodKG = pid_sgva[4];
              JsonConstants.hoodConstants.hoodKV = pid_sgva[5];
              JsonConstants.hoodConstants.hoodKA = pid_sgva[6];

              motor.setGains(
                  JsonConstants.hoodConstants.hoodKP,
                  JsonConstants.hoodConstants.hoodKI,
                  JsonConstants.hoodConstants.hoodKD,
                  JsonConstants.hoodConstants.hoodKS,
                  JsonConstants.hoodConstants.hoodKG,
                  JsonConstants.hoodConstants.hoodKV,
                  JsonConstants.hoodConstants.hoodKA);
            },
            hoodKP,
            hoodKI,
            hoodKD,
            hoodKS,
            hoodKG,
            hoodKV,
            hoodKA);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (expoConstraintsVA) -> {
              motor.setProfileConstraints(
                  MotionProfileConfig.immutable(
                      RotationsPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero().div(Seconds.of(1.0)),
                      Volts.of(expoConstraintsVA[0]).div(RotationsPerSecond.of(1)),
                      Volts.of(expoConstraintsVA[1]).div(RotationsPerSecondPerSecond.of(1))));

              JsonConstants.hoodConstants.hoodExpoKV = expoConstraintsVA[0];
              JsonConstants.hoodConstants.hoodExpoKA = expoConstraintsVA[1];
            },
            hoodExpoKV,
            hoodExpoKA);

        // Make sure that the mechanism doesn't break itself by controlling outside of its safe
        // range of motion, even in test mode
        clampAndControlToAngle(Degrees.of(hoodTuningSetpointDegrees.getAsDouble()));
      }
      case HoodCurrentTuning -> {
        motor.controlOpenLoopCurrent(Amps.of(hoodTuningAmps.getAsDouble()));
      }
      case HoodVoltageTuning -> {
        motor.controlOpenLoopVoltage(Volts.of(hoodTuningVolts.getAsDouble()));
      }
      default -> {}
    }
  }

  /**
   * Returns `true` if the absolute value of the hood's velocity is greater than or equal to the
   * hood homing movement threshold, `false` otherwise
   *
   * @return `true` if the absolute value of the hood's velocity is greater than or equal to the
   *     hood homing movement threshold, `false` otherwise
   */
  protected boolean isMoving() {
    final AngularVelocityUnit velocityComparisonUnit = RadiansPerSecond;

    return getVelocity().abs(velocityComparisonUnit)
        >= JsonConstants.turretConstants.homingMovementThreshold.in(velocityComparisonUnit);
  }

  /**
   * Sets the hood's current position to the homed position. Should be called whenever homing states
   * determine that the system is at its homed position.
   */
  protected void onHomePositionReached() {
    motor.setCurrentPosition(JsonConstants.hoodConstants.minHoodAngle);
  }

  /**
   * Updates the hood subsystem on whether the homing switch is pressed. This should only be called
   * by a coordinator/supervisor-layer action scheduled with the DependencyOrderedExecutor.
   *
   * @param isHomingSwitchPressed True if the homing switch is pressed (hood should assume it has
   *     homed), false if the switch isn't pressed.
   */
  public void setIsHomingSwitchPressed(boolean isHomingSwitchPressed) {
    this.isHomingSwitchPressed = isHomingSwitchPressed;
  }

  /**
   * Returns whether or not the homing switch is currently pressed (or was pressed when its inputs
   * were last read from hardware.)
   *
   * @return True if the homing switch is pressed (the mechanism should consider itself homed),
   *     false if not
   */
  protected boolean isHomingSwitchPressed() {
    return isHomingSwitchPressed;
  }

  /**
   * Apply the homing voltage defined in HoodConstants to gently home the hood into its bottom
   * hardstop
   */
  protected void applyHomingVoltage() {
    motor.controlOpenLoopVoltage(JsonConstants.hoodConstants.homingVoltage);
  }

  /**
   * Get the mechanism velocity of the hood
   *
   * @return An AngularVelocity representing the velocity of the physical hood
   */
  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadiansPerSecond);
  }

  /** Applies a CoastOut/neutral request. */
  protected void coast() {
    motor.controlCoast();
  }

  protected void controlToGoalPitch() {
    Logger.recordOutput("Hood/goalPitchRadians", goalPitch.in(Radians));
    Angle goalAngle =
        Degrees.of(90)
            .minus(goalPitch)
            .minus(JsonConstants.hoodConstants.mechanismAngleToExitAngle);
    Logger.recordOutput("Hood/goalAngleRadians", goalAngle.in(Radians));
    clampAndControlToAngle(goalAngle);
  }

  protected void controlToGoalAngle() {
    Logger.recordOutput("Hood/goalAngleRadians", goalAngle.in(Radians));
    clampAndControlToAngle(goalAngle);
  }

  /**
   * Given a target angle, clamp it to be within the allowed bounds and then control the mechanism
   * toward it
   *
   * @param goalAngle The Angle to target
   */
  private void clampAndControlToAngle(Angle goalAngle) {
    Angle clampedGoalAngle =
        UnitUtils.clampMeasure(
            goalAngle,
            JsonConstants.hoodConstants.minHoodAngle,
            JsonConstants.hoodConstants.maxHoodAngle);
    Logger.recordOutput("Hood/clampedGoalAngleRadians", goalAngle.in(Radians));
    motor.controlToPositionExpoProfiled(clampedGoalAngle);
  }

  /**
   * Get the current fuel exit angle based on the position of the hood
   *
   * @return An Angle representing the current exit angle of a fuel being shot from the hood.
   */
  public Angle getCurrentExitAngle() {
    return Degrees.of(90)
        .minus(
            Radians.of(inputs.positionRadians)
                .plus(JsonConstants.hoodConstants.mechanismAngleToExitAngle));
  }

  public boolean isHoodTestMode() {
    return testModeManager.isInTestMode();
  }

  /**
   * Sets the goal exit angle of the hood. Note that this value is NOT the goal angle of the hood,
   * but rather the desired fuel exit angle while shooting.
   *
   * <p>This method updates the hood's current action, so that as soon as homing is completed, it
   * will target the pitch requested. This means that it can safely be called at any time,
   * regardless of homing status.
   *
   * @param goalPitch An Angle containing the desired angle above the horizon (zero being horizontal
   *     and 90 degrees being a vertical shot) at which a fuel should exit the hood.
   */
  public void targetPitch(Angle goalPitch) {
    this.requestedAction = HoodAction.TargetPitch;
    this.goalPitch.mut_replace(goalPitch);
  }

  /**
   * Sets the goal angle of the hood. Note that this is a hood angle, NOT an exit angle for the
   * projectile.
   *
   * <p>This method updates the hood's current action, so that as soon as homing is completed, it
   * will target the angle requested. This means that it can safely be called at any time,
   * regardless of homing status.
   *
   * @param angleRadians A double containing the desired hood angle in radians.
   */
  public void targetAngleRadians(double angleRadians) {
    this.requestedAction = HoodAction.TargetAngle;
    this.goalAngle.mut_replace(angleRadians, Radians);
  }
}
