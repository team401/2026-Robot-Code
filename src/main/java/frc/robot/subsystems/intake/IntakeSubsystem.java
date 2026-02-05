package frc.robot.subsystems.intake;

import coppercore.controls.state_machine.StateMachine;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.JsonConstants;
import frc.robot.util.TestModeManager;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  TestModeManager<PivotTestMode> pivotTestModeManager =
      new TestModeManager<PivotTestMode>("IntakePivot", PivotTestMode.class);

  TestModeManager<RollerTestMode> rollerTestModeManager =
      new TestModeManager<RollerTestMode>("IntakeRollers", RollerTestMode.class);

  StateMachine<IntakeSubsystem> intakeStateMachine;

  public enum PivotPosition {
    STOWED,
    DEPLOYED
  }

  protected MotorIO pivotMotorIO;
  protected MotorIO rollersLeadMotorIO;
  protected MotorIO rollersFollowerMotorIO;

  protected MotorInputsAutoLogged pivotInputs;
  protected MotorInputsAutoLogged rollerLeadMotorInputs;
  protected MotorInputsAutoLogged rollerFollowerMotorInputs;

  protected PivotPosition targetPivotPosition = PivotPosition.STOWED;

  public IntakeSubsystem(
      MotorIO pivotMotorIO, MotorIO rollersLeadMotorIO, MotorIO rollersFollowerMotorIO) {
    this.pivotMotorIO = pivotMotorIO;
    this.rollersLeadMotorIO = rollersLeadMotorIO;
    this.rollersFollowerMotorIO = rollersFollowerMotorIO;

    this.pivotInputs = new MotorInputsAutoLogged();
    this.rollerLeadMotorInputs = new MotorInputsAutoLogged();
    this.rollerFollowerMotorInputs = new MotorInputsAutoLogged();

    this.intakeStateMachine = new StateMachine<IntakeSubsystem>(this);

    this.intakeStateMachine.registerState(IntakeState.testModeState);
    this.intakeStateMachine.registerState(IntakeState.deployedState);
    this.intakeStateMachine.registerState(IntakeState.stowedState);
    this.intakeStateMachine.registerState(IntakeState.waitForButtonState);
    this.intakeStateMachine.registerState(IntakeState.homingWaitForMovementState);
    this.intakeStateMachine.registerState(IntakeState.homingWaitForStopMovingState);
    this.intakeStateMachine.registerState(IntakeState.homingDoneState);

    IntakeState.homingWaitForMovementState
        .whenFinished()
        .transitionTo(IntakeState.homingWaitForMovementState);
    IntakeState.homingWaitForMovementState
        .whenTimeout(JsonConstants.intakeConstants.homingTimeoutSeconds)
        .transitionTo(IntakeState.homingDoneState);

    IntakeState.homingWaitForStopMovingState
        .whenFinished()
        .transitionTo(IntakeState.homingDoneState);

    IntakeState.homingDoneState
        .when(IntakeState::shouldBeInTestMode, "If should be in test mode")
        .transitionTo(IntakeState.testModeState);
    IntakeState.homingDoneState
        .whenFinished()
        .andWhen(() -> this.targetPivotPosition == PivotPosition.STOWED, "Target Stowed")
        .transitionTo(IntakeState.stowedState);
    IntakeState.homingDoneState
        .whenFinished()
        .andWhen(() -> this.targetPivotPosition == PivotPosition.DEPLOYED, "Target Deployed")
        .transitionTo(IntakeState.deployedState);

    IntakeState.testModeState
        .whenFinished()
        .andWhen(() -> this.targetPivotPosition == PivotPosition.STOWED, "Target Stowed")
        .transitionTo(IntakeState.stowedState);
    IntakeState.testModeState
        .whenFinished()
        .andWhen(() -> this.targetPivotPosition == PivotPosition.DEPLOYED, "Target Deployed")
        .transitionTo(IntakeState.deployedState);

    IntakeState.waitForButtonState
        .whenFinished()
        .andWhen(() -> this.targetPivotPosition == PivotPosition.STOWED, "Target Stowed")
        .transitionTo(IntakeState.stowedState);
    IntakeState.waitForButtonState
        .whenFinished()
        .andWhen(() -> this.targetPivotPosition == PivotPosition.DEPLOYED, "Target Deployed")
        .transitionTo(IntakeState.deployedState);
    IntakeState.waitForButtonState
        .when(DriverStation::isEnabled, "When robot is enabled")
        .transitionTo(IntakeState.homingDoneState);

    IntakeState.deployedState
        .whenFinished()
        .andWhen(() -> this.targetPivotPosition == PivotPosition.STOWED, "Target Stowed")
        .transitionTo(IntakeState.stowedState);
    IntakeState.stowedState
        .whenFinished()
        .andWhen(() -> this.targetPivotPosition == PivotPosition.DEPLOYED, "Target Deployed")
        .transitionTo(IntakeState.deployedState);

    this.intakeStateMachine.setState(IntakeState.stowedState);
  }

  public void runRollers(AngularVelocity rollerSpeed) {
    rollersLeadMotorIO.controlToVelocityUnprofiled(rollerSpeed);
  }

  public void stopRollers() {
    rollersLeadMotorIO.controlNeutral();
  }

  public void setTargetPivotPosition(PivotPosition position) {
    this.targetPivotPosition = position;
  }

  protected void setTargetPivotAngle(Angle angle) {
    pivotMotorIO.controlToPositionUnprofiled(angle);
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
    //  TODO: implement roller speed tuning periodic
  }
}
