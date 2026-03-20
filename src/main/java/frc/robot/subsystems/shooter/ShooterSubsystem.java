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
import coppercore.wpilib_interface.subsystems.motors.MotorIO.GainSlot;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CoordinationLayer.ShotMode;
import frc.robot.DependencyOrderedExecutor;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.shooter.ShooterState.CoastState;
import frc.robot.subsystems.shooter.ShooterState.TestModeState;
import frc.robot.subsystems.shooter.ShooterState.VelocityControlState;
import frc.robot.util.LoggedTunablePIDGains;
import frc.robot.util.StateMachineDump;
import frc.robot.util.TestModeManager;
import frc.robot.util.TotalCurrentCalculator;
import frc.robot.util.math.Lazy;
import org.apache.commons.math3.stat.regression.SimpleRegression;
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
  Lazy<LoggedTunablePIDGains> slot0TunableGains =
      new Lazy<>(
          () ->
              new LoggedTunablePIDGains(
                  "ShooterTunables/slot0", JsonConstants.shooterConstants.shooterSlot0Gains));
  Lazy<LoggedTunablePIDGains> slot1TunableGains =
      new Lazy<>(
          () ->
              new LoggedTunablePIDGains(
                  "ShooterTunables/slot1", JsonConstants.shooterConstants.shooterSlot1Gains));

  Lazy<LoggedTunableNumber> slot0EpsilonRPM =
      new Lazy<>(
          () ->
              new LoggedTunableNumber(
                  "ShooterTunables/slot0EpsilonRPM",
                  JsonConstants.shooterConstants.shooterSlot0Epsilon.in(RPM)));

  Lazy<LoggedTunableNumber> shooterMaxVelocityRPM;
  Lazy<LoggedTunableNumber> shooterMaxAccelerationRPMPerSecond;

  Lazy<LoggedTunableNumber> shooterTuningRPM;
  Lazy<LoggedTunableNumber> shooterTuningAmps;
  Lazy<LoggedTunableNumber> shooterTuningVolts;

  // State variables
  private final MutAngularVelocity targetVelocity = RPM.mutable(0.0);

  @AutoLogOutput(key = "Shooter/requestedAction")
  private ShooterAction requestedAction = ShooterAction.Coast;

  private final Debouncer isAtGoalVelocityDebouncer =
      new Debouncer(
          JsonConstants.shooterConstants.atSetpointDebounceTime.in(Seconds), DebounceType.kFalling);

  // State variables for FF characterization
  private SimpleRegression ffRegression = new SimpleRegression();
  private final Timer characterizationTimer = new Timer();

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
            () -> requestedAction == ShooterAction.ControlVelocity,
            "requestedAction == ControlVelocity")
        .transitionTo(velocityControlState);

    coastState
        .when(() -> testModeManager.isInTestMode(), "Is shooter test mode")
        .transitionTo(testModeState);

    velocityControlState
        .when(() -> requestedAction == ShooterAction.Coast, "requestedAction == Coast")
        .transitionTo(coastState);

    velocityControlState
        .when(() -> testModeManager.isInTestMode(), "Is shooter test mode")
        .transitionTo(testModeState);

    testModeState
        .when(
            () ->
                !testModeManager.isInTestMode() && requestedAction == ShooterAction.ControlVelocity,
            "Is not shooter test mode and requestedAction == ControlVelocity")
        .transitionTo(velocityControlState);

    testModeState
        .when(
            () -> !testModeManager.isInTestMode() && requestedAction == ShooterAction.Coast,
            "Is not shooter test mode and requestedAction == Coast")
        .transitionTo(coastState);

    stateMachine.setState(velocityControlState);
    StateMachineDump.write("shooter", stateMachine);

    // Initialize tunable numbers for test modes
    shooterMaxVelocityRPM =
        new Lazy<>(
            () ->
                new LoggedTunableNumber(
                    "ShooterTunables/shooterMaxVelocityRPM",
                    JsonConstants.shooterConstants.shooterMaxVelocity.in(RPM)));
    shooterMaxAccelerationRPMPerSecond =
        new Lazy<>(
            () ->
                new LoggedTunableNumber(
                    "ShooterTunables/shooterMaxAccelerationRPMPerSecond",
                    JsonConstants.shooterConstants.shooterMaxAcceleration.in(RPM.per(Second))));

    shooterTuningRPM =
        new Lazy<>(() -> new LoggedTunableNumber("ShooterTunables/shooterTuningRPM", 0.0));
    shooterTuningAmps =
        new Lazy<>(() -> new LoggedTunableNumber("ShooterTunables/shooterTuningAmps", 0.0));
    shooterTuningVolts =
        new Lazy<>(() -> new LoggedTunableNumber("ShooterTunables/shooterTuningVolts", 0.0));

    AutoLogOutputManager.addObject(this);

    dependencyOrderedExecutor.registerAction(UPDATE_INPUTS, this::updateInputs);
  }

  private void updateInputs() {
    leadMotor.updateInputs(leadMotorInputs);
    followerMotor.updateInputs(followerMotorInputs);

    TotalCurrentCalculator.reportCurrent(
        hashCode(), leadMotorInputs.supplyCurrentAmps + followerMotorInputs.supplyCurrentAmps);

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

    GainSlot slot =
        Units.radiansPerSecondToRotationsPerMinute(leadMotorInputs.velocityRadiansPerSecond)
                > Units.radiansPerSecondToRotationsPerMinute(leadMotorInputs.closedLoopReference)
                    - JsonConstants.shooterConstants.shooterSlot0Epsilon.in(RPM)
            ? GainSlot.Slot0
            : GainSlot.Slot1;
    leadMotor.selectGainSlot(slot);
    Logger.recordOutput("Shooter/gainSlot", slot);

    stateMachine.periodic();

    if (stateMachine.getCurrentState() != coastState) {
      followerMotor.follow(
          JsonConstants.canBusAssignment.shooterLeaderId,
          JsonConstants.shooterConstants.invertFollower);
    }
  }

  /** Runs when test mode is entered. Should be called by the test mode state. */
  protected void testInit() {
    switch (testModeManager.getTestMode()) {
      case ShooterFFCharacterization -> {
        this.ffRegression.clear();
        this.characterizationTimer.restart();

        Logger.recordOutput("Shooter/Characterization/kS", 0.0);
        Logger.recordOutput("Shooter/Characterization/kV", 0.0);
      }
      default -> {}
    }
  }

  protected void testPeriodic() {
    switch (testModeManager.getTestMode()) {
      case ShooterClosedLoopTuning -> {
        slot0TunableGains
            .get()
            .ifChanged(
                hashCode(),
                (gains) -> {
                  JsonConstants.shooterConstants.shooterSlot0Gains = gains;
                  leadMotor.setGains(
                      GainSlot.Slot0,
                      gains.kP(),
                      gains.kI(),
                      gains.kD(),
                      gains.kS(),
                      gains.kG(),
                      gains.kV(),
                      gains.kA());
                });

        slot1TunableGains
            .get()
            .ifChanged(
                hashCode(),
                (gains) -> {
                  JsonConstants.shooterConstants.shooterSlot1Gains = gains;
                  leadMotor.setGains(
                      GainSlot.Slot1,
                      gains.kP(),
                      gains.kI(),
                      gains.kD(),
                      gains.kS(),
                      gains.kG(),
                      gains.kV(),
                      gains.kA());
                });

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
            shooterMaxVelocityRPM.get(),
            shooterMaxAccelerationRPMPerSecond.get());

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (slot0EpsilonRPM) -> {
              JsonConstants.shooterConstants.shooterSlot0Epsilon = RPM.of(slot0EpsilonRPM[0]);
            },
            slot0EpsilonRPM.get());

        leadMotor.controlToVelocityProfiled(RPM.of(shooterTuningRPM.get().getAsDouble()));
      }
      case ShooterCurrentTuning -> {
        leadMotor.controlOpenLoopCurrent(Amps.of(shooterTuningAmps.get().getAsDouble()));
      }
      case ShooterVoltageTuning -> {
        leadMotor.controlOpenLoopVoltage(Volts.of(shooterTuningVolts.get().getAsDouble()));
      }
      case ShooterFFCharacterization -> {
        if (DriverStation.isEnabled()) {
          Current characterizationCurrent =
              (Current)
                  JsonConstants.shooterConstants.characterizationRampRate.times(
                      Seconds.of(characterizationTimer.get()));
          double characterizationCurrentAmps = characterizationCurrent.in(Amps);

          // TODO: Figure out why velocity is negative in sim
          double velocityRotationsPerSecond =
              Math.abs(Units.radiansToRotations(leadMotorInputs.velocityRadiansPerSecond));

          ffRegression.addData(velocityRotationsPerSecond, characterizationCurrentAmps);
          double kS = ffRegression.getIntercept();
          double kV = ffRegression.getSlope();

          Logger.recordOutput("Shooter/Characterization/sampleCount", ffRegression.getN());
          Logger.recordOutput(
              "Shooter/Characterization/appliedCurrentAmps", characterizationCurrentAmps);
          Logger.recordOutput("Shooter/Characterization/kS", kS);
          Logger.recordOutput("Shooter/Characterization/kV", kV);

          leadMotor.controlOpenLoopCurrent(characterizationCurrent);
        }
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
    requestedAction = ShooterAction.ControlVelocity;
  }

  /**
   * Get the current velocity of the shooter, as reported by the leader and follower motors'
   * internal encoder velocity values.
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
    requestedAction = ShooterAction.Coast;
  }

  /**
   * Returns whether or not the shooter is within the velocity threshold of its goal velocity
   *
   * <p>Returns false if the shooter is commanded to stop.
   *
   * @return {@code true} if the shooter is controlling to a velocity and its measured velocity is
   *     within the threshold of its target velocity, {@code false} otherwise.
   */
  public boolean isAtGoalVelocity(ShotMode shotMode) {
    AngularVelocity threshold =
        switch (shotMode) {
          case Hub -> JsonConstants.shooterConstants.shooterVelocitySetpointEpsilon;
          case Pass -> JsonConstants.shooterConstants.shooterPassingVelocitySetpointEpsilon;
        };

    boolean isAtGoalVelocity =
        isAtGoalVelocityDebouncer.calculate(
            requestedAction == ShooterAction.ControlVelocity
                && getVelocity().isNear(targetVelocity, threshold));

    Logger.recordOutput("Shooter/isAtGoalVelocity", isAtGoalVelocity);
    return isAtGoalVelocity;
  }
}
