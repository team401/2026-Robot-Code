package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.robot.DependencyOrderedExecutor;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.shooter.ShooterState.CoastState;
import frc.robot.subsystems.shooter.ShooterState.TestModeState;
import frc.robot.subsystems.shooter.ShooterState.VelocityControlState;
import frc.robot.util.TestModeManager;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * A shooter powered by 3 kraken x60s running closed-loop velocity control with a Motion Magic
 * Velocity profile.
 */
public class ShooterSubsystem extends MonitoredSubsystem {
  private enum ShooterAction {
    Coast,
    ControlVelocity
  }

  public static final ActionKey UPDATE_INPUTS = new ActionKey("ShooterSubsystem::updateInputs");

  private final TestModeManager<TestMode> testModeManager =
      new TestModeManager<>("Shooter", TestMode.class);

  // Motors and inputs
  private final MotorInputsAutoLogged leadMotorInputs = new MotorInputsAutoLogged();
  private final MotorInputsAutoLogged followerMotorInputs = new MotorInputsAutoLogged();

  private final MotorIO leadMotor;
  private final MotorIO followerMotor;

  // State machine and states
  private final StateMachine<ShooterSubsystem> stateMachine;

  private final ShooterState coastState;
  private final ShooterState velocityControlState;
  private final ShooterState testModeState;

  // Tunable Numbers
  LoggedTunableNumber shooterKP;
  LoggedTunableNumber shooterKI;
  LoggedTunableNumber shooterKD;

  LoggedTunableNumber shooterKS;
  LoggedTunableNumber shooterKV;
  LoggedTunableNumber shooterKA;

  LoggedTunableNumber shooterMaxVelocityRPM;
  LoggedTunableNumber shooterMaxAccelerationRPMPerSecond;

  LoggedTunableNumber shooterTuningRPM;
  LoggedTunableNumber shooterTuningAmps;
  LoggedTunableNumber shooterTuningVolts;

  // State variables
  private final MutAngularVelocity targetVelocity = RPM.mutable(0.0);

  @AutoLogOutput(key = "Shooter/currentAction")
  private ShooterAction currentAction = ShooterAction.Coast;

  public ShooterSubsystem(
      DependencyOrderedExecutor dependencyOrderedExecutor,
      MotorIO leadMotor,
      MotorIO followerMotor) {
    this.leadMotor = leadMotor;
    this.followerMotor = followerMotor;

    leadMotor.setRequestUpdateFrequency(JsonConstants.shooterConstants.shooterClosedLoopFrequency);
    followerMotor.setRequestUpdateFrequency(
        JsonConstants.shooterConstants.shooterClosedLoopFrequency);

    stateMachine = new StateMachine<>(this);

    coastState = stateMachine.registerState(new CoastState());
    velocityControlState = stateMachine.registerState(new VelocityControlState());
    testModeState = stateMachine.registerState(new TestModeState());

    coastState
        .when(
            () -> currentAction == ShooterAction.ControlVelocity,
            "currentAction == ControlVelocity")
        .transitionTo(velocityControlState);

    coastState
        .when(() -> testModeManager.isInTestMode(), "Is shooter test mode")
        .transitionTo(testModeState);

    velocityControlState
        .when(() -> currentAction == ShooterAction.Coast, "currentAction == Coast")
        .transitionTo(coastState);

    velocityControlState
        .when(() -> testModeManager.isInTestMode(), "Is shooter test mode")
        .transitionTo(testModeState);

    testModeState
        .when(() -> !testModeManager.isInTestMode(), "Is not shooter test mode")
        .transitionTo(velocityControlState);

    stateMachine.setState(velocityControlState);

    // Initialize tunable numbers for test modes
    shooterKP =
        new LoggedTunableNumber(
            "ShooterTunables/shooterKP", JsonConstants.shooterConstants.shooterKP);
    shooterKI =
        new LoggedTunableNumber(
            "ShooterTunables/shooterKI", JsonConstants.shooterConstants.shooterKI);
    shooterKD =
        new LoggedTunableNumber(
            "ShooterTunables/shooterKD", JsonConstants.shooterConstants.shooterKD);

    shooterKS =
        new LoggedTunableNumber(
            "ShooterTunables/shooterKS", JsonConstants.shooterConstants.shooterKS);
    shooterKV =
        new LoggedTunableNumber(
            "ShooterTunables/shooterKV", JsonConstants.shooterConstants.shooterKV);
    shooterKA =
        new LoggedTunableNumber(
            "ShooterTunables/shooterKA", JsonConstants.shooterConstants.shooterKA);

    shooterMaxVelocityRPM =
        new LoggedTunableNumber(
            "ShooterTunables/shooterMaxVelocityRPM",
            JsonConstants.shooterConstants.shooterMaxVelocity.in(RPM));
    shooterMaxAccelerationRPMPerSecond =
        new LoggedTunableNumber(
            "ShooterTunables/shooterMaxAccelerationRPMPerSecond",
            JsonConstants.shooterConstants.shooterMaxAcceleration.in(RPM.per(Second)));

    shooterTuningRPM = new LoggedTunableNumber("ShooterTunables/shooterTuningRPM", 0.0);
    shooterTuningAmps = new LoggedTunableNumber("ShooterTunables/shooterTuningAmps", 0.0);
    shooterTuningVolts = new LoggedTunableNumber("ShooterTunables/shooterTuningVolts", 0.0);

    AutoLogOutputManager.addObject(this);

    dependencyOrderedExecutor.registerAction(UPDATE_INPUTS, this::updateInputs);
  }

  private void updateInputs() {
    leadMotor.updateInputs(leadMotorInputs);
    followerMotor.updateInputs(followerMotorInputs);

    Logger.processInputs("Shooter/LeadMotorInputs", leadMotorInputs);
    Logger.processInputs("Shooter/FollowerMotorInputs", followerMotorInputs);

    Logger.recordOutput(
        "Shooter/ClosedLoopReferenceRadiansPerSecond", leadMotorInputs.closedLoopReference);
    Logger.recordOutput(
        "Shooter/ClosedLoopReferenceSlopeRadiansPerSecondPerSecond",
        leadMotorInputs.closedLoopReferenceSlope);
  }

  @Override
  public void monitoredPeriodic() {
    Logger.recordOutput("Shooter/TargetVelocityRadPerSec", targetVelocity.in(RadiansPerSecond));

    stateMachine.periodic();

    if (stateMachine.getCurrentState() != coastState) {
      followerMotor.follow(
          JsonConstants.canBusAssignment.shooterLeaderId,
          JsonConstants.shooterConstants.invertFollower);
    }
  }

  protected void testPeriodic() {
    switch (testModeManager.getTestMode()) {
      case ShooterClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sva) -> {
              JsonConstants.shooterConstants.shooterKP = pid_sva[0];
              JsonConstants.shooterConstants.shooterKI = pid_sva[1];
              JsonConstants.shooterConstants.shooterKD = pid_sva[2];
              JsonConstants.shooterConstants.shooterKS = pid_sva[3];
              JsonConstants.shooterConstants.shooterKV = pid_sva[4];
              JsonConstants.shooterConstants.shooterKA = pid_sva[5];

              leadMotor.setGains(
                  JsonConstants.shooterConstants.shooterKP,
                  JsonConstants.shooterConstants.shooterKI,
                  JsonConstants.shooterConstants.shooterKD,
                  JsonConstants.shooterConstants.shooterKS,
                  0.0,
                  JsonConstants.shooterConstants.shooterKV,
                  JsonConstants.shooterConstants.shooterKA);
            },
            shooterKP,
            shooterKI,
            shooterKD,
            shooterKS,
            shooterKV,
            shooterKA);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (maxProfile) -> {
              JsonConstants.shooterConstants.shooterMaxVelocity = RPM.of(maxProfile[0]);
              JsonConstants.shooterConstants.shooterMaxAcceleration =
                  RPM.per(Second).of(maxProfile[1]);

              leadMotor.setProfileConstraints(
                  MotionProfileConfig.immutable(
                      JsonConstants.shooterConstants.shooterMaxVelocity,
                      JsonConstants.shooterConstants.shooterMaxAcceleration,
                      RotationsPerSecondPerSecond.zero().div(Seconds.of(1.0)),
                      Volts.zero().div(RPM.of(1.0)),
                      Volts.zero().div(RotationsPerSecondPerSecond.of(1.0))));
            },
            shooterMaxVelocityRPM,
            shooterMaxAccelerationRPMPerSecond);

        leadMotor.controlToVelocityProfiled(RPM.of(shooterTuningRPM.getAsDouble()));
      }
      case ShooterCurrentTuning -> {
        leadMotor.controlOpenLoopCurrent(Amps.of(shooterTuningAmps.getAsDouble()));
      }
      case ShooterVoltageTuning -> {
        leadMotor.controlOpenLoopVoltage(Volts.of(shooterTuningVolts.getAsDouble()));
      }
      default -> {}
    }
  }

  protected void controlToTargetVelocity() {
    leadMotor.controlToVelocityProfiled(targetVelocity);
  }

  protected void coast() {
    leadMotor.controlCoast();
    followerMotor.controlCoast();
  }

  /**
   * Sets the shooter's target velocity, in RPM
   *
   * <p>This method should only be called by the coordination layer
   *
   * @param velocityRPM A double containing target velocity, in RPM
   */
  public void setTargetVelocityRPM(double velocityRPM) {
    targetVelocity.mut_replace(velocityRPM, RPM);
    currentAction = ShooterAction.ControlVelocity;
  }

  /**
   * Get the current velocity of the shooter, as reported by average of the the leader and follower
   * motor's internal encoder velocity values.
   *
   * @return The arithmetic mean of the two shooter motors' velocity estimates in radians per second
   */
  public double getVelocityRadiansPerSecond() {
    return (leadMotorInputs.velocityRadiansPerSecond + followerMotorInputs.velocityRadiansPerSecond)
        / 2;
  }

  public AngularVelocity getVelocity() {
    // Optimization: this could be changed to continually mut_replace into a mutable measure to
    // avoid allocating an object every cycle later if performance is a concern.
    return RadiansPerSecond.of(getVelocityRadiansPerSecond());
  }

  /**
   * Stops the shooter by coasting it to a stop.
   *
   * <p>This method should only be called by the coordination layer.
   */
  public void stopShooter() {
    currentAction = ShooterAction.Coast;
  }

  /**
   * Returns whether or not the shooter is within the velocity threshold of its goal velocity
   *
   * <p>Returns false if the shooter is commanded to stop.
   *
   * @return {@code true} if the shooter is controlling to a velocity and its measured velocity is
   *     within the threshold of its target velocity, {@code false} otherwise.
   */
  @AutoLogOutput(key = "Shooter/isAtGoalVelocity")
  public boolean isAtGoalVelocity() {
    return currentAction == ShooterAction.ControlVelocity
        && getVelocity()
            .isNear(targetVelocity, JsonConstants.shooterConstants.shooterVelocitySetpointEpsilon);
  }
}
