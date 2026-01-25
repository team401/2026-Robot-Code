package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitorWithAlert.MonitorWithAlertBuilder;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.DependencyOrderedExecutor;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.hood.HoodState.HomingWaitForButtonState;
import frc.robot.subsystems.hood.HoodState.HomingWaitForMovementState;
import frc.robot.subsystems.hood.HoodState.HomingWaitForStoppingState;
import frc.robot.subsystems.hood.HoodState.IdleState;
import frc.robot.subsystems.hood.HoodState.TestModeState;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * The HoodSubsystem class defines the Hood subsystem, which controls the hardware for the hood on
 * the aimer of our shooter superstructure. It uses simple closed loop control to accomplish its
 * tasks.
 */
public class HoodSubsystem extends MonitoredSubsystem {
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  public static final ActionKey UPDATE_INPUTS = new ActionKey("HoodSubsystem::updateInputs");

  /**
   * The HoodDependencies class contains input values that the hood depends on getting from other
   * subsystems/the supervisor layer
   */
  public static class HoodDependencies {
    /**
     * Whether or not the homing switch is currently pressed. This value should default to false
     * when a homing limit switch is not present.
     */
    private boolean isHomingSwitchPressed = false;

    public boolean isHomingSwitchPressed() {
      return isHomingSwitchPressed;
    }
  }

  private final HoodDependencies dependencies = new HoodDependencies();

  // State machine and states
  private final StateMachine<HoodSubsystem> stateMachine;

  private final HoodState homingWaitForButtonState;
  private final HoodState homingWaitForMovementState;
  private final HoodState homingWaitForStoppingState;
  private final HoodState idleState;
  private final HoodState testModeState;

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

  public HoodSubsystem(MotorIO motor) {
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

    homingWaitForButtonState =
        (HoodState) stateMachine.registerState(new HomingWaitForButtonState());
    homingWaitForMovementState =
        (HoodState) stateMachine.registerState(new HomingWaitForMovementState());
    homingWaitForStoppingState =
        (HoodState) stateMachine.registerState(new HomingWaitForStoppingState());
    idleState = (HoodState) stateMachine.registerState(new IdleState());
    testModeState = (HoodState) stateMachine.registerState(new TestModeState());

    homingWaitForButtonState.whenFinished().transitionTo(idleState);
    homingWaitForButtonState
        .when(() -> DriverStation.isEnabled(), "Robot is enabled")
        .transitionTo(homingWaitForMovementState);

    homingWaitForMovementState.whenFinished().transitionTo(homingWaitForStoppingState);
    homingWaitForMovementState
        .whenTimeout(JsonConstants.hoodConstants.homingMaxUnmovingTime)
        .transitionTo(homingWaitForStoppingState);

    homingWaitForStoppingState.whenFinished().transitionTo(idleState);

    idleState.when(() -> isHoodTestMode(), "Is hood test mode").transitionTo(testModeState);

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
        new LoggedTunableNumber("HoodTunables/HoodTuningSetpointDegrees", 0.0);
    hoodTuningAmps = new LoggedTunableNumber("HoodTunables/HoodTuningAmps", 0.0);
    hoodTuningVolts = new LoggedTunableNumber("HoodTunables/HoodTuningVolts", 0.0);

    var dependencyOrderedExecutor = DependencyOrderedExecutor.getDefaultInstance();

    dependencyOrderedExecutor.registerAction(UPDATE_INPUTS, this::updateInputs);

    AutoLogOutputManager.addObject(this);
  }

  private void updateInputs() {
    motor.updateInputs(inputs);
    Logger.processInputs("Hood/inputs", inputs);
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
    switch (TestModeManager.getTestMode()) {
      case HoodClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sgva) -> {
              motor.setGains(
                  pid_sgva[0],
                  pid_sgva[1],
                  pid_sgva[2],
                  pid_sgva[3],
                  pid_sgva[4],
                  pid_sgva[5],
                  pid_sgva[6]);
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
            (maxProfile) -> {
              motor.setProfileConstraints(
                  MotionProfileConfig.immutable(
                      RotationsPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero().div(Seconds.of(1.0)),
                      Volts.of(maxProfile[0]).div(RotationsPerSecond.of(1)),
                      Volts.of(maxProfile[1]).div(RotationsPerSecondPerSecond.of(1))));
            },
            hoodExpoKV,
            hoodExpoKA);

        motor.controlToPositionExpoProfiled(Degrees.of(hoodTuningSetpointDegrees.getAsDouble()));
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
   * Sets the hood's current position to the homed position. Should be called whenever homing states
   * determine that the system is at its homed position.
   */
  protected void homePositionReached() {
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
    dependencies.isHomingSwitchPressed = isHomingSwitchPressed;
  }

  /**
   * Get the dependencies object associated with this HoodSubsystem instance. This method provides a
   * way for hood states to access the dependency values of the hood.
   *
   * @return The HoodDependencies object that this Hood uses/contains
   */
  protected HoodDependencies getDependencies() {
    return dependencies;
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
  public void coast() {
    motor.controlCoast();
  }

  public boolean isHoodTestMode() {
    return switch (TestModeManager.getTestMode()) {
      case HoodClosedLoopTuning, HoodCurrentTuning, HoodVoltageTuning, HoodPhoenixTuning -> true;
      default -> false;
    };
  }
}
