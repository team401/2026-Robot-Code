package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.JsonConstants;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.TestModeManager;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

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
            "IntakeRollersTargetSpeedRPM",
            JsonConstants.intakeConstants.intakeRollerSpeed.in(
                RPM)); // Default to the intake roller speed defined in constants

    this.intakeStateMachine = new StateMachine<IntakeSubsystem>(this);

    IntakeState.testModeState = this.intakeStateMachine.registerState(new IntakeState.TestModeState(this));
    List.of(
      IntakeState.controlToPositionState,      IntakeState.waitForButtonState, 
      IntakeState.homingWaitForMovementState,  IntakeState.homingWaitForStopMovingState,
      IntakeState.homingDoneState)
        .forEach(this.intakeStateMachine::registerState);



    // ### Test Mode Transitions
    // Any time we finish any state and we should be in test mode, we transition to the test mode state at
    // the soonest time we can without disrupting the the homing process. So that when we are in test mode,
    // we can be sure that it has been properly homed and that the setpoints we get in test mode are accurate.
    IntakeState.waitForButtonState
        .whenFinished().andWhen(IntakeState::shouldBeInTestMode, "Should be in test mode")
        .transitionTo(IntakeState.testModeState);
    IntakeState.homingDoneState
        .whenFinished().andWhen(IntakeState::shouldBeInTestMode, "Should be in test mode")
        .transitionTo(IntakeState.testModeState);
    IntakeState.controlToPositionState
        .when(IntakeState::shouldBeInTestMode, "Should be in test mode")
        .transitionTo(IntakeState.testModeState);
    IntakeState.testModeState
        .whenFinished()
        .transitionTo(IntakeState.controlToPositionState);


    // ### Homing Button Transitions
    IntakeState.waitForButtonState
        .whenFinished()
        .transitionTo(IntakeState.controlToPositionState);
    IntakeState.waitForButtonState
        .when(DriverStation::isEnabled, "When robot is enabled and button has not been pressed")
        .transitionTo(IntakeState.homingWaitForMovementState);

    // ### Wait for movement transitions
    // If the mechanism starts moving, we assume that it is has started the homing
    // process properly and we transition to the homing wait for stop moving state
    // to wait for it stop moving to finish the homing process
    IntakeState.homingWaitForMovementState
        .whenFinished()
        .transitionTo(IntakeState.homingWaitForMovementState);
    // If the mechanism doesn't start moving within the timeout, we assume that it
    // is already in the homed position and we finish the homing process by
    // transitioning to the homing done state
    IntakeState.homingWaitForMovementState
        .whenTimeout(JsonConstants.intakeConstants.homingTimeoutSeconds)
        .transitionTo(IntakeState.homingDoneState);


    
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
    IntakeState.homingDoneState
        .whenFinished()
        .transitionTo(IntakeState.controlToPositionState);

    this.intakeStateMachine.setState(IntakeState.waitForButtonState);
  }

  public void runRollers(AngularVelocity rollerSpeed) {
    rollersLeadMotorIO.controlToVelocityUnprofiled(rollerSpeed);
  }

  public void stopRollers() {
    rollersLeadMotorIO.controlNeutral();
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

  protected void controlToTargetPivotAngle() {
    pivotMotorIO.controlToPositionUnprofiled(this.targetPivotAngle);
  }

  public void periodic() {
    pivotMotorIO.updateInputs(pivotInputs);
    rollersLeadMotorIO.updateInputs(rollerLeadMotorInputs);
    rollersFollowerMotorIO.updateInputs(rollerFollowerMotorInputs);

    Logger.processInputs("intake/pivot/inputs", pivotInputs);
    Logger.processInputs("intake/rollerLead/inputs", rollerLeadMotorInputs);
    Logger.processInputs("intake/rollerFollower/inputs", rollerFollowerMotorInputs);

    if (rollerTestModeManager.getTestMode() == RollerTestMode.RollerSpeedTuning) {
      rollerSpeedTuningPeriodic();
    }

    intakeStateMachine.periodic();
  }

  public void rollerSpeedTuningPeriodic() {
    var speed = LoggedTunableMeasure.ANGULAR_VELOCITY.of(
        "IntakeRollerSpeedRPM",
        JsonConstants.intakeConstants.intakeRollerSpeed.in(RPM),
        RPM);


    speed.ifChanged(rollerSpeed -> JsonConstants.intakeConstants.intakeRollerSpeed = rollerSpeed);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        rollerSpeed -> JsonConstants.intakeConstants.intakeRollerSpeed = RPM.of(rollerSpeed[0]),
        rollersTargetSpeedTunable);
  }

}
