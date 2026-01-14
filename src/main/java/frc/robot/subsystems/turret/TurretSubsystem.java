package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.turret.states.HomingWaitForMovementState;
import frc.robot.subsystems.turret.states.HomingWaitForStoppingState;
import frc.robot.subsystems.turret.states.IdleState;
import frc.robot.subsystems.turret.states.TestModeState;
import org.littletonrobotics.junction.AutoLogOutput;

// TODO: Add/Improve Javadocs
/**
 * A single motor turret.
 *
 * <p>Assume all angles are clockwise-positive since that's what physics & math use.
 */
public class TurretSubsystem extends MonitoredSubsystem {
  // Motor and inputs
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  // Dependencies (these are what we would have fetched using extensive supplier networks in 2025
  // and before)
  public static class TurretDependencies {
    /**
     * Whether or not the homing switch is currently pressed. This value should default to false
     * when a homing limit switch is not present.
     */
    public boolean isHomingSwitchPressed = false;
  }

  private final TurretDependencies dependencies = new TurretDependencies();

  // State machine and states
  private final StateMachine<TurretSubsystem> stateMachine;

  private final State<TurretSubsystem> homingWaitForMovementState;
  private final State<TurretSubsystem> homingWaitForStoppingState;
  private final State<TurretSubsystem> idleState;
  private final State<TurretSubsystem> testModeState;

  // Subsystem state ("normal" member variables)
  /**
   * Has the turret been homed yet? Applying any closed-loop request before the turret has been
   * homed is very dangerous and should not be allowed.
   */
  private boolean hasBeenHomed = false;

  // Tunable numbers
  LoggedTunableNumber turretKP;
  LoggedTunableNumber turretKI;
  LoggedTunableNumber turretKD;

  LoggedTunableNumber turretKS;
  LoggedTunableNumber turretKV;
  LoggedTunableNumber turretKA;
  LoggedTunableNumber turretKG;

  LoggedTunableNumber turretExpoKV;
  LoggedTunableNumber turretExpoKA;

  LoggedTunableNumber turretTuningSetpointDegrees;
  LoggedTunableNumber turretTuningAmps;
  LoggedTunableNumber turretTuningVolts;

  public TurretSubsystem(MotorIO motor) {
    this.motor = motor;

    // Define state machine transitions, register states
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

    // Initialize tunable numbers for test modes
    turretKP =
        new LoggedTunableNumber("TurretTunables/turretKP", JsonConstants.turretConstants.turretKP);
    turretKI =
        new LoggedTunableNumber("TurretTunables/turretKI", JsonConstants.turretConstants.turretKI);
    turretKD =
        new LoggedTunableNumber("TurretTunables/turretKD", JsonConstants.turretConstants.turretKD);

    turretKS =
        new LoggedTunableNumber("TurretTunables/turretKS", JsonConstants.turretConstants.turretKS);
    turretKV =
        new LoggedTunableNumber("TurretTunables/turretKV", JsonConstants.turretConstants.turretKV);
    turretKA =
        new LoggedTunableNumber("TurretTunables/turretKA", JsonConstants.turretConstants.turretKA);

    turretExpoKV =
        new LoggedTunableNumber(
            "TurretTunables/turretExpoKV", JsonConstants.turretConstants.turretExpoKV);
    turretExpoKA =
        new LoggedTunableNumber(
            "TurretTunables/turretExpoKA", JsonConstants.turretConstants.turretExpoKA);

    turretTuningSetpointDegrees =
        new LoggedTunableNumber("TurretTunables/turretTuningSetpointDegrees", 0.0);
    turretTuningAmps = new LoggedTunableNumber("TurretTunables/turretTuningAmps", 0.0);
    turretTuningVolts = new LoggedTunableNumber("TurretTunables/turretTuningVolts", 0.0);
  }

  @Override
  public void monitoredPeriodic() {
    motor.updateInputs(inputs);

    stateMachine.periodic();
  }

  /**
   * Polls for test-mode specific actions (for example, updating PIDs).
   *
   * <p>This method MUST be called in periodic
   */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case TurretClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sva) -> {
              motor.setGains(
                  pid_sva[0], pid_sva[1], pid_sva[2], pid_sva[3], 0, pid_sva[4], pid_sva[5]);
            },
            turretKP,
            turretKI,
            turretKD,
            turretKS,
            turretKV,
            turretKA);

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
            turretExpoKA,
            turretExpoKV);

        motor.controlToPositionExpoProfiled(
            turretToMotorAngle(Degrees.of(turretTuningSetpointDegrees.getAsDouble())));
      }
      case TurretCurrentTuning -> {
        motor.controlOpenLoopCurrent(Amps.of(turretTuningAmps.getAsDouble()));
      }
      case TurretVoltageTuning -> {
        motor.controlOpenLoopVoltage(Volts.of(turretTuningVolts.getAsDouble()));
      }
      default -> {}
    }
  }

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

  @AutoLogOutput(key = "Turret/robotRelativePosition")
  public Angle getTurretAngleRobotRelative() {
    return Radians.of(motorToTurret(inputs.positionRadians));
  }

  public AngularVelocity getTurretVelocity() {
    return RadiansPerSecond.of(motorToTurret(inputs.velocityRadiansPerSecond));
  }

  protected void setPositionToHomedPosition() {
    hasBeenHomed = true;
    motor.setCurrentPosition(turretToMotorAngle(JsonConstants.turretConstants.homingAngle));
  }

  protected void coast() {
    motor.controlCoast();
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
