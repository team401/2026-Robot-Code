package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.turret.states.HomingWaitForMovementState;
import frc.robot.subsystems.turret.states.HomingWaitForStoppingState;
import frc.robot.subsystems.turret.states.IdleState;
import frc.robot.subsystems.turret.states.TestModeState;

// TODO: Javadocs
public class TurretSubsystem extends MonitoredSubsystem {
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  public static class TurretDependencies {
    /**
     * Whether or not the homing switch is currently pressed. This value should default to false
     * when a homing limit switch is not present.
     */
    public boolean isHomingSwitchPressed = false;
  }

  private final TurretDependencies dependencies = new TurretDependencies();

  private final StateMachine<TurretSubsystem> stateMachine;

  private final State<TurretSubsystem> homingWaitForMovementState;
  private final State<TurretSubsystem> homingWaitForStoppingState;
  private final State<TurretSubsystem> idleState;
  private final State<TurretSubsystem> testModeState;

  public TurretSubsystem(MotorIO motor) {
    this.motor = motor;

    stateMachine = new StateMachine<>(this);

    homingWaitForMovementState = stateMachine.registerState(new HomingWaitForMovementState());
    homingWaitForStoppingState = stateMachine.registerState(new HomingWaitForStoppingState());
    idleState = stateMachine.registerState(new IdleState());
    testModeState = stateMachine.registerState(new TestModeState());

    homingWaitForMovementState.whenFinished().transitionTo(homingWaitForStoppingState);

    // If it hits the timeout for never moving, kick into "wait for stopping state" which will
    // immediately detect that it has stopped moving and home the system.
    homingWaitForMovementState
        .whenTimeout(JsonConstants.turretConstants.homingMaxUnmovingTime)
        .transitionTo(homingWaitForMovementState);

    homingWaitForStoppingState.whenFinished().transitionTo(idleState);

    idleState
        .when(TurretSubsystem::isTurretTestMode, "In turret test mode")
        .transitionTo(testModeState);

    testModeState
        .when((turret) -> !turret.isTurretTestMode(), "Not in turret test mode")
        .transitionTo(idleState);

    stateMachine.setState(homingWaitForMovementState);
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);

    stateMachine.periodic();
  }

  /**
   * Polls for test-mode specific actions (for example, updating PIDs).
   *
   * <p>This method MUST be called by RobotContainer, as it does NOT run automatically
   */
  public void testPeriodic() {}

  public TurretDependencies getDependenciesObject() {
    return this.dependencies;
  }

  protected void applyHomingVoltage() {
    motor.controlOpenLoopVoltage(JsonConstants.turretConstants.homingVoltage);
  }

  private double motorToTurret(double motorValue) {
    return motorValue / JsonConstants.turretConstants.turretReduction;
  }

  private Angle motorToTurretAngle(Angle motorAngle) {
    return motorAngle.div(JsonConstants.turretConstants.turretReduction);
  }

  private double turretToMotor(double turretValue) {
    return turretValue * JsonConstants.turretConstants.turretReduction;
  }

  private Angle turretToMotorAngle(Angle turretAngle) {
    return turretAngle.times(JsonConstants.turretConstants.turretReduction);
  }

  public Angle getTurretAngleRobotRelative() {
    return Radians.of(motorToTurret(inputs.positionRadians));
  }

  public AngularVelocity getTurretVelocity() {
    return RadiansPerSecond.of(motorToTurret(inputs.velocityRadiansPerSecond));
  }

  protected void setPositionToHomedPosition() {
    motor.setCurrentPosition(turretToMotorAngle(JsonConstants.turretConstants.homingAngle));
  }

  /**
   * Check TestModeManager for whether or not the currently selected test mode requires the turret
   * to switch to its tuning state.
   *
   * @return True if the robot is enabled in test mode with a turret test mode selected, false if a
   *     non-turret test mode is selected, the test mode doesn't require the turret to enter
   *     TestModeState, the robot isn't enabled in test mode, or TestModeManager hasn't been
   *     initialized.
   */
  private boolean isTurretTestMode() {
    return switch (TestModeManager.getTestMode()) {
      case TurretClosedLoopTuning, TurretCurrentTuning, TurretVoltageTuning -> true;
      default -> false;
    };
  }
}
