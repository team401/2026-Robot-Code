package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.JsonConstants;
import frc.robot.util.TestModeManager;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends MonitoredSubsystem {

  TestModeManager<PivotTestMode> pivotTestModeManager =
      new TestModeManager<PivotTestMode>("IntakePivot", PivotTestMode.class);

  TestModeManager<RollerTestMode> rollerTestModeManager =
      new TestModeManager<RollerTestMode>("IntakeRollers", RollerTestMode.class);

  StateMachine<IntakeSubsystem> intakeStateMachine;

  protected MotorIO pivotMotorIO;
  protected MotorIO rollersLeadMotorIO;
  protected MotorIO rollersFollowerMotorIO;

  protected MotorInputsAutoLogged pivotInputs;
  protected MotorInputsAutoLogged rollerLeadMotorInputs;
  protected MotorInputsAutoLogged rollerFollowerMotorInputs;

  protected Angle targetPivotAngle = Degrees.zero();

  private LoggedTunableNumber rollersTargetSpeedTunable;

  private IntakeDependencies dependencies = new IntakeDependencies();

  // Dependencies (these are what we would have fetched using extensive supplier networks in 2025
  // and before)
  public static class IntakeDependencies {
    /**
     * Whether or not the homing switch is currently pressed. This value should default to false
     * when a homing limit switch is not present.
     */
    private boolean isHomingSwitchPressed = false;

    public boolean isHomingSwitchPressed() {
      return isHomingSwitchPressed;
    }
  }

  public IntakeDependencies getDependencies() {
    return this.dependencies;
  }

  public void setIsHomingSwitchPressed(boolean isHomingSwitchPressed) {
    dependencies.isHomingSwitchPressed = isHomingSwitchPressed;
  }

  public IntakeSubsystem(
      MotorIO pivotMotorIO, MotorIO rollersLeadMotorIO, MotorIO rollersFollowerMotorIO) {
    this.pivotMotorIO = pivotMotorIO;
    this.rollersLeadMotorIO = rollersLeadMotorIO;
    this.rollersFollowerMotorIO = rollersFollowerMotorIO;

    this.pivotInputs = new MotorInputsAutoLogged();
    this.rollerLeadMotorInputs = new MotorInputsAutoLogged();
    this.rollerFollowerMotorInputs = new MotorInputsAutoLogged();

    this.rollersTargetSpeedTunable =
        new LoggedTunableNumber(
            "IntakeTunables/RollersTargetSpeedRPM",
            JsonConstants.intakeConstants.intakeRollerSpeed.in(RPM));

    this.intakeStateMachine = new StateMachine<IntakeSubsystem>(this);

    IntakeState.testModeState =
        this.intakeStateMachine.registerState(new IntakeState.TestModeState(this));
    List.of(
            IntakeState.controlToPositionState,
            IntakeState.waitForButtonState,
            IntakeState.homingWaitForMovementState,
            IntakeState.homingWaitForStopMovingState,
            IntakeState.homingDoneState)
        .forEach(this.intakeStateMachine::registerState);

    // ### Test Mode Transitions
    // Any time we finish any state and we should be in test mode, we transition to the test mode
    // state at
    // the soonest time we can without disrupting the homing process. So that when we are in
    // test mode,
    // we can be sure that it has been properly homed and that the setpoints we get in test mode are
    // accurate.
    IntakeState.waitForButtonState
        .whenFinished()
        .andWhen(IntakeState::shouldBeInTestMode, "Should be in test mode")
        .transitionTo(IntakeState.testModeState);
    IntakeState.homingDoneState
        .whenFinished()
        .andWhen(IntakeState::shouldBeInTestMode, "Should be in test mode")
        .transitionTo(IntakeState.testModeState);
    IntakeState.controlToPositionState
        .when(IntakeState::shouldBeInTestMode, "Should be in test mode")
        .transitionTo(IntakeState.testModeState);
    IntakeState.testModeState.whenFinished().transitionTo(IntakeState.controlToPositionState);

    // ### Homing Button Transitions
    IntakeState.waitForButtonState.whenFinished().transitionTo(IntakeState.controlToPositionState);
    IntakeState.waitForButtonState
        .when(DriverStation::isEnabled, "When robot is enabled and button has not been pressed")
        .transitionTo(IntakeState.homingWaitForMovementState);

    // ### Wait for movement transitions
    // If the robot gets disabled during the homing process, we transition back to
    // the wait for button state to wait for the operator to re-enable the robot
    //  and restart the homing process.
    IntakeState.homingWaitForMovementState
        .when(DriverStation::isDisabled, "When robot is disabled during homing")
        .transitionTo(IntakeState.waitForButtonState);
    // If the mechanism starts moving, we assume that it is has started the homing
    // process properly and we transition to the homing wait for stop moving state
    // to wait for it stop moving to finish the homing process
    IntakeState.homingWaitForMovementState
        .whenFinished()
        .transitionTo(IntakeState.homingWaitForStopMovingState);
    // If the mechanism doesn't start moving within the timeout, we assume that it
    // is already in the homed position and we finish the homing process by
    // transitioning to the homing done state
    IntakeState.homingWaitForMovementState
        .whenTimeout(JsonConstants.intakeConstants.homingTimeoutSeconds)
        .transitionTo(IntakeState.homingDoneState);

    // If the robot gets disabled during the homing process, we transition back to
    // the wait for button state to wait for the operator to re-enable the robot
    //  and restart the homing process.
    IntakeState.homingWaitForStopMovingState
        .when(DriverStation::isDisabled, "When robot is disabled during homing")
        .transitionTo(IntakeState.waitForButtonState);
    // If the mechanism starts moving, we assume that we have started the homing
    // process properly and so we wait for it to stop moving by hitting a hard
    // stop. Once it stops moving, we assume that we are in the homed position
    // and we finish the homing process by transitioning to the homing done state
    IntakeState.homingWaitForStopMovingState
        .whenFinished()
        .transitionTo(IntakeState.homingDoneState);

    // ### Exiting homing process transition
    // Once the mechanism has stopped moving, we consider the homing process to be
    // done and we set the current position as zero and transition to the control
    // to position state
    IntakeState.homingDoneState.whenFinished().transitionTo(IntakeState.controlToPositionState);

    this.intakeStateMachine.setState(IntakeState.waitForButtonState);
  }

  public void runRollers(AngularVelocity rollerSpeed) {
    rollersLeadMotorIO.controlToVelocityUnprofiled(rollerSpeed);
  }

  public void stopRollers() {
    rollersLeadMotorIO.controlNeutral();
    // We don't need to command the follower motor to stop because
    // it is always following the lead motor
  }

  public void setTargetPivotAngle(Angle angle) {
    this.targetPivotAngle = angle;
  }

  public Angle getCurrentTargetPivotAngle() {
    return this.targetPivotAngle;
  }

  public Angle getCurrentPivotAngle() {
    return Radians.of(this.pivotInputs.positionRadians);
  }

  // Should these be blocked from executing if we are in test mode?

  public void setTargetPositionStowed() {
    setTargetPivotAngle(JsonConstants.intakeConstants.stowPositionAngle);
  }

  public void setTargetPositionIntaking() {
    setTargetPivotAngle(JsonConstants.intakeConstants.intakePositionAngle);
  }

  @Override
  public void monitoredPeriodic() {
    pivotMotorIO.updateInputs(pivotInputs);
    rollersLeadMotorIO.updateInputs(rollerLeadMotorInputs);
    rollersFollowerMotorIO.updateInputs(rollerFollowerMotorInputs);

    Logger.processInputs("Intake/Pivot/Inputs", pivotInputs);
    Logger.processInputs("Intake/RollerLead/Inputs", rollerLeadMotorInputs);
    Logger.processInputs("Intake/RollerFollower/Inputs", rollerFollowerMotorInputs);

    // This is run outside of the test mode because we want to be able to tune the roller speed
    // in test mode and be able to control the pivot as if we were operating the robot normally
    if (rollerTestModeManager.getTestMode() == RollerTestMode.RollerSpeedTuning) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          rollerSpeed -> JsonConstants.intakeConstants.intakeRollerSpeed = RPM.of(rollerSpeed[0]),
          rollersTargetSpeedTunable);
    }

    Logger.recordOutput("Intake/State", intakeStateMachine.getCurrentState().getName());

    intakeStateMachine.periodic();

    // Ensure that even if we accidentally command the follower motor to do something
    // it won't cause any issues because we always command it to follow the lead motor
    // at the end of the periodic
    rollersFollowerMotorIO.follow(JsonConstants.canBusAssignment.intakeRollersLeadMotorId, false);
  }

  protected void controlToTargetPivotAngle() {
    pivotMotorIO.controlToPositionUnprofiled(this.targetPivotAngle);
  }

  protected void zeroPositionIfBelowZero() {
    if (pivotInputs.positionRadians < 0) {
      // pivotMotorIO.setCurrentPositionAsZero();
    }
  }
}
