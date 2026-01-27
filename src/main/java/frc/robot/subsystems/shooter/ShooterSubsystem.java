package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
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
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.robot.DependencyOrderedExecutor;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.shooter.ShooterState.TestModeState;
import frc.robot.subsystems.shooter.ShooterState.VelocityControlState;
import frc.robot.util.TestModeManager;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * A shooter powered by 3 kraken x60s running closed-loop velocity control with a Motion Magic
 * Velocity profile.
 */
public class ShooterSubsystem extends MonitoredSubsystem {
  public static final ActionKey UPDATE_INPUTS = new ActionKey("ShooterSubsystem::updateInputs");

  private final TestModeManager<TestMode> testModeManager =
      new TestModeManager<>("Shooter", TestMode.class);

  // Motors and inputs
  private final MotorInputsAutoLogged leadMotorInputs = new MotorInputsAutoLogged();
  private final MotorInputsAutoLogged topFollowerMotorInputs = new MotorInputsAutoLogged();
  private final MotorInputsAutoLogged bottomFollowerMotorInputs = new MotorInputsAutoLogged();

  private final MotorIO leadMotor;
  private final MotorIO topFollowerMotor;
  private final MotorIO bottomFollowerMotor;

  // State machine and states
  private final StateMachine<ShooterSubsystem> stateMachine;

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

  public ShooterSubsystem(
      DependencyOrderedExecutor dependencyOrderedExecutor,
      MotorIO leadMotor,
      MotorIO topFollowerMotor,
      MotorIO bottomFollowerMotor) {
    this.leadMotor = leadMotor;
    this.topFollowerMotor = topFollowerMotor;
    this.bottomFollowerMotor = bottomFollowerMotor;

    stateMachine = new StateMachine<>(this);

    velocityControlState = stateMachine.registerState(new VelocityControlState());
    testModeState = stateMachine.registerState(new TestModeState());

    velocityControlState
        .when(shooter -> shooter.isShooterTestMode(), "Is shooter test mode")
        .transitionTo(testModeState);

    testModeState
        .when(shooter -> !shooter.isShooterTestMode(), "Is not shooter test mode")
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
    topFollowerMotor.updateInputs(topFollowerMotorInputs);
    bottomFollowerMotor.updateInputs(bottomFollowerMotorInputs);

    Logger.processInputs("Shooter/LeadMotorInputs", leadMotorInputs);
    Logger.processInputs("Shooter/TopFollowerMotorInputs", topFollowerMotorInputs);
    Logger.processInputs("Shooter/BottomFollowerInputs", bottomFollowerMotorInputs);

    Logger.recordOutput(
        "Shooter/ClosedLoopReferenceRadiansPerSecond", leadMotorInputs.closedLoopReference);
    Logger.recordOutput(
        "Shooter/ClosedLoopReferenceSlopeRadiansPerSecondPerSecond",
        leadMotorInputs.closedLoopReferenceSlope);
  }

  @Override
  public void monitoredPeriodic() {
    Logger.recordOutput("Shooter/TargetVelocity", targetVelocity);

    stateMachine.periodic();
  }

  protected void testPeriodic() {
    switch (testModeManager.getTestMode()) {
      case ShooterClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid_sva) -> {
              leadMotor.setGains(
                  pid_sva[0], pid_sva[1], pid_sva[2], pid_sva[3], 0, pid_sva[4], pid_sva[5]);
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
              leadMotor.setProfileConstraints(
                  MotionProfileConfig.immutable(
                      RPM.of(maxProfile[0]),
                      RPM.per(Second).of(maxProfile[1]),
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

  public boolean isShooterTestMode() {
    return switch (testModeManager.getTestMode()) {
      case ShooterClosedLoopTuning,
          ShooterCurrentTuning,
          ShooterPhoenixTuning,
          ShooterVoltageTuning ->
          true;
      default -> false;
    };
  }
}
