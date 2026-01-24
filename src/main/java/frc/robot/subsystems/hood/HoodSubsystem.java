package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.MonitorWithAlert.MonitorWithAlertBuilder;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

    stateMachine.setState(homingWaitForButtonState);

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
  protected void testPeriodic() {}

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
