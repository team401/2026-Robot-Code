package frc.robot.subsystems.turret;

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
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.UnitUtils;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.DependencyOrderedExecutor;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.turret.TurretState.HomingWaitForButtonState;
import frc.robot.subsystems.turret.TurretState.HomingWaitForMovementState;
import frc.robot.subsystems.turret.TurretState.HomingWaitForStoppingState;
import frc.robot.subsystems.turret.TurretState.IdleState;
import frc.robot.subsystems.turret.TurretState.TestModeState;
import frc.robot.subsystems.turret.TurretState.TrackHeadingState;
import frc.robot.util.AngleUtil;
import frc.robot.util.TestModeManager;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

// TODO: Add/Improve Javadocs
/**
 * A single motor turret.
 *
 * <p>Assume all angles are counterclockwise-positive since that's what physics & math use.
 */
public class TurretSubsystem extends MonitoredSubsystem {
  private enum TurretAction {
    /** Do nothing, coast the turret and wait */
    Idle,
    /** Track the heading supplied to the turret as its goal heading */
    TrackHeading
  }

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
    private boolean isHomingSwitchPressed = false;

    /** The current field-centric heading of the robot, according to the drivetrain pose estimate */
    private Rotation2d robotHeading = Rotation2d.kZero;

    public boolean isHomingSwitchPressed() {
      return isHomingSwitchPressed;
    }

    public Rotation2d getRobotHeading() {
      return robotHeading;
    }
  }

  public static final ActionKey UPDATE_INPUTS = new ActionKey("TurretSubsystem::updateInputs");

  private final TurretDependencies dependencies = new TurretDependencies();

  // State machine and states
  private final StateMachine<TurretSubsystem> stateMachine;

  private final TurretState homingWaitForButtonState;
  private final TurretState homingWaitForMovementState;
  private final TurretState homingWaitForStoppingState;
  private final TurretState idleState;
  private final TurretState trackHeadingState;
  private final TurretState testModeState;

  // Tunable numbers
  LoggedTunableNumber turretKP;
  LoggedTunableNumber turretKI;
  LoggedTunableNumber turretKD;

  LoggedTunableNumber turretKS;
  LoggedTunableNumber turretKV;
  LoggedTunableNumber turretKA;

  LoggedTunableNumber turretExpoKV;
  LoggedTunableNumber turretExpoKA;

  LoggedTunableNumber turretTuningSetpointDegrees;
  LoggedTunableNumber turretTuningAmps;
  LoggedTunableNumber turretTuningVolts;

  TestModeManager<TestMode> testModeManager =
      new TestModeManager<TestMode>("Turret", TestMode.class);

  // State variables
  /**
   * The last action requested of the turret. This is different from a state, which is what the
   * turret is currently doing by necessity (e.g. homing).
   */
  @AutoLogOutput(key = "Turret/action")
  private TurretAction requestedAction = TurretAction.Idle;

  /**
   * The field-centric goal heading of the turret to track when in TrackingState. This value should
   * be updated by the coordinator layer via setGoalHeading.
   */
  @AutoLogOutput(key = "Turret/goalHeading")
  private Rotation2d goalTurretHeading = Rotation2d.kZero;

  public TurretSubsystem(DependencyOrderedExecutor dependencyOrderedExecutor, MotorIO motor) {
    this.motor = motor;

    // Define state machine transitions, register states
    stateMachine = new StateMachine<>(this);

    homingWaitForButtonState = stateMachine.registerState(new HomingWaitForButtonState());
    homingWaitForMovementState = stateMachine.registerState(new HomingWaitForMovementState());
    homingWaitForStoppingState = stateMachine.registerState(new HomingWaitForStoppingState());
    idleState = stateMachine.registerState(new IdleState());
    trackHeadingState = stateMachine.registerState(new TrackHeadingState());
    testModeState = stateMachine.registerState(new TestModeState());

    homingWaitForButtonState.whenFinished().transitionTo(idleState);
    homingWaitForButtonState
        .when(turret -> DriverStation.isEnabled(), "Robot is enabled")
        .transitionTo(homingWaitForMovementState);

    homingWaitForMovementState.whenFinished().transitionTo(homingWaitForStoppingState);

    // If it hits the timeout for never moving, kick into "wait for stopping state" which will
    // immediately detect that it has stopped moving and home the system.
    homingWaitForMovementState
        .whenTimeout(JsonConstants.turretConstants.homingMaxUnmovingTime)
        .transitionTo(homingWaitForStoppingState);

    homingWaitForStoppingState.whenFinished().transitionTo(idleState);

    idleState
        .when(
            turret -> turret.requestedAction == TurretAction.TrackHeading, "Action == TrackHeading")
        .transitionTo(trackHeadingState);

    idleState
        .when(turret -> turret.isTurretTestMode(), "In turret test mode")
        .transitionTo(testModeState);

    trackHeadingState
        .when(
            turret -> turret.requestedAction != TurretAction.TrackHeading, "Action != TrackHeading")
        .transitionTo(idleState);

    testModeState
        .when(turret -> !turret.isTurretTestMode(), "Not in turret test mode")
        .transitionTo(idleState);

    stateMachine.setState(homingWaitForButtonState);

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

    // Add turret to the AutoLogOutputManager, as, being stored in an optional, it won't be visible
    // to the recursive search of Robot's fields
    AutoLogOutputManager.addObject(this);

    dependencyOrderedExecutor.registerAction(UPDATE_INPUTS, this::updateInputs);
  }

  public void updateInputs() {
    motor.updateInputs(inputs);
    Logger.processInputs("Turret/inputs", inputs);

    Logger.recordOutput("Turret/closedLoopReferenceRadians", inputs.closedLoopReference);
    Logger.recordOutput(
        "Turret/closedLoopReferenceSlopeRadPerSec", inputs.closedLoopReferenceSlope);
  }

  @Override
  public void monitoredPeriodic() {
    Logger.recordOutput("Turret/State", stateMachine.getCurrentState().getName());
    stateMachine.periodic();
    Logger.recordOutput("Turret/StateAfter", stateMachine.getCurrentState().getName());
  }

  /**
   * Polls for test-mode specific actions (for example, updating PIDs).
   *
   * <p>This method MUST be called in periodic by the TestModeState
   */
  protected void testPeriodic() {
    switch (testModeManager.getTestMode()) {
      case TurretClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sva) -> {
              JsonConstants.turretConstants.turretKP = pid_sva[0];
              JsonConstants.turretConstants.turretKI = pid_sva[1];
              JsonConstants.turretConstants.turretKD = pid_sva[2];
              JsonConstants.turretConstants.turretKS = pid_sva[3];
              JsonConstants.turretConstants.turretKV = pid_sva[4];
              JsonConstants.turretConstants.turretKA = pid_sva[5];
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
              JsonConstants.turretConstants.turretExpoKV = maxProfile[0];
              JsonConstants.turretConstants.turretExpoKA = maxProfile[1];
              motor.setProfileConstraints(
                  MotionProfileConfig.immutable(
                      RotationsPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero(),
                      RotationsPerSecondPerSecond.zero().div(Seconds.of(1.0)),
                      Volts.of(maxProfile[0]).div(RotationsPerSecond.of(1)),
                      Volts.of(maxProfile[1]).div(RotationsPerSecondPerSecond.of(1))));
            },
            turretExpoKV,
            turretExpoKA);

        motor.controlToPositionExpoProfiled(Degrees.of(turretTuningSetpointDegrees.getAsDouble()));
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

  public TurretDependencies getDependencies() {
    return this.dependencies;
  }

  protected void applyHomingVoltage() {
    motor.controlOpenLoopVoltage(JsonConstants.turretConstants.homingVoltage);
  }

  @AutoLogOutput(key = "Turret/robotRelativePosition")
  public Angle getTurretAngleRobotRelative() {
    return Radians.of(inputs.positionRadians);
  }

  public AngularVelocity getTurretVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadiansPerSecond);
  }

  protected void setPositionToHomedPosition() {
    motor.setCurrentPosition(JsonConstants.turretConstants.homingAngle);
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
    return switch (testModeManager.getTestMode()) {
      case TurretClosedLoopTuning, TurretCurrentTuning, TurretVoltageTuning, TurretPhoenixTuning ->
          true;
      default -> false;
    };
  }

  private void controlToTurretCentricPosition(Angle goalAngleTurretCentric) {
    Logger.recordOutput("Turret/GoalAngle", goalAngleTurretCentric);
    Angle clampedGoalAngle =
        UnitUtils.clampMeasure(
            goalAngleTurretCentric,
            JsonConstants.turretConstants.minTurretAngle,
            JsonConstants.turretConstants.maxTurretAngle);
    Logger.recordOutput("Turret/ClampedGoalAngle", clampedGoalAngle);

    motor.controlToPositionExpoProfiled(clampedGoalAngle);
  }

  /**
   * Control to the current goal heading, based on the current robot heading
   *
   * <p>Converts the current goal heading into a turret angle by subtracting the drivetrain heading
   * and then applying the heading offset from TurretConstants
   */
  protected void controlToGoalHeading() {
    Rotation2d robotRelativeHeading = goalTurretHeading.minus(dependencies.robotHeading);
    Rotation2d turretRelativeHeading =
        AngleUtil.normalizeHeading(
            robotRelativeHeading.plus(
                new Rotation2d(JsonConstants.turretConstants.headingToTurretAngle)));

    controlToTurretCentricPosition(turretRelativeHeading.getMeasure());
  }

  /**
   * Calculates the current field centric turret heading, by converting turret heading to a
   * robot-centric heading and then adjusting for robot heading.
   *
   * @return A Rotation2d representing the current field-centric heading of the turret.
   */
  @AutoLogOutput(key = "Turret/FieldCentricTurretHeading")
  public Rotation2d getFieldCentricTurretHeading() {
    return new Rotation2d(
            getTurretAngleRobotRelative().minus(JsonConstants.turretConstants.headingToTurretAngle))
        .plus(dependencies.robotHeading);
  }

  /**
   * Updates the turret subsystem on whether the homing switch is pressed. This should only be
   * called by a coordinator/supervisor-layer action scheduled with the DependencyOrderedExecutor.
   *
   * @param isHomingSwitchPressed True if the homing switch is pressed (turret should assume it has
   *     homed), false if the switch isn't pressed.
   */
  public void setIsHomingSwitchPressed(boolean isHomingSwitchPressed) {
    dependencies.isHomingSwitchPressed = isHomingSwitchPressed;
  }

  /**
   * Update the turret subsystem on the robot's current heading. This should only be called by a
   * coordinator/supervisor-layer action scheduled with the DependencyOrderedExecutor to update
   * turret inputs.
   *
   * @param robotHeading A Rotation2d, the heading of the robot from the drivetrain's pose estimate.
   */
  public void setRobotHeading(Rotation2d robotHeading) {
    dependencies.robotHeading = robotHeading;
  }

  /**
   * Sets the turret's field centric goal heading. This method should only be called by the
   * coordination layer.
   *
   * <p>This method updates the turret's current action to track the goal heading. This means that,
   * if no other state is in the way (e.g. homing), the turret will immediately begin tracking the
   * heading next time periodic is run.
   *
   * @param goalHeading A Rotation2d containing the field-centric goal heading
   */
  public void targetGoalHeading(Rotation2d goalHeading) {
    this.requestedAction = TurretAction.TrackHeading;
    this.goalTurretHeading = goalHeading;
  }
}
