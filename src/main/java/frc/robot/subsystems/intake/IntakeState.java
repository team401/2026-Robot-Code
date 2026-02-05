package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import coppercore.parameter_tools.LoggedTunableNumber;
import edu.wpi.first.units.Units;
import frc.robot.constants.JsonConstants;
import frc.robot.util.LoggedTunablePIDGains;
import frc.robot.util.PIDGains;

public class IntakeState {

  public static boolean shouldBeInTestMode(IntakeSubsystem world) {
    if (world.pivotTestModeManager.getTestMode() == PivotTestMode.None) {
      return false;
    }
    if (world.rollerTestModeManager.getTestMode() != RollerTestMode.None) {
      return false;
    }
    if (world.pivotTestModeManager.getTestMode() != PivotTestMode.None) {
      return false;
    }
    return true;
  }

  public static State<IntakeSubsystem> testModeState =
      new State<IntakeSubsystem>("TestMode") {

        private static LoggedTunableNumber pivotTuningVoltage;
        private static LoggedTunableNumber rollerTuningVoltage;
        private static LoggedTunableNumber pivotTuningCurrent;
        private static LoggedTunableNumber rollerTuningCurrent;
        private static LoggedTunableNumber pivotTuningSetpointDegrees;
        private static LoggedTunableNumber rollerTuningSetpointRPM;

        private static LoggedTunablePIDGains pivotTuningGains;
        private static LoggedTunablePIDGains rollerTuningGains;

        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {

          pivotTestPeriodic(world.pivotTestModeManager.getTestMode(), world);
          rollerTestPeriodic(world.rollerTestModeManager.getTestMode(), world);

          if (!shouldBeInTestMode(world)) {
            finish();
          }
        }

        public void pivotTestPeriodic(PivotTestMode testMode, IntakeSubsystem world) {
          switch (testMode) {
            case PivotVoltageTuning:
              LoggedTunableNumber.ifChanged(
                  hashCode(),
                  (voltage) -> world.pivotMotorIO.controlOpenLoopVoltage(Volts.of(voltage[0])),
                  pivotTuningVoltage);
              break;
            case PivotCurrentTuning:
              LoggedTunableNumber.ifChanged(
                  hashCode(),
                  (current) -> world.pivotMotorIO.controlOpenLoopCurrent(Amps.of(current[0])),
                  pivotTuningCurrent);
              break;
            case PivotClosedLoopTuning:
              pivotTuningGains.ifChanged(
                  hashCode(),
                  pivotTuningGains.getMotorIOApplier(
                      world.pivotMotorIO,
                      (PIDGains gains) -> JsonConstants.intakeConstants.pivotPIDGains = gains));
              LoggedTunableNumber.ifChanged(
                  hashCode(),
                  (setpoint) -> world.setTargetPivotAngle(Degrees.of(setpoint[0])),
                  pivotTuningSetpointDegrees);
              break;
            default:
              break;
          }
        }

        public void rollerTestPeriodic(RollerTestMode testMode, IntakeSubsystem world) {
          switch (testMode) {
            case RollerVoltageTuning:
              LoggedTunableNumber.ifChanged(
                  hashCode(),
                  (voltage) ->
                      world.rollersLeadMotorIO.controlOpenLoopVoltage(Volts.of(voltage[0])),
                  rollerTuningVoltage);
              break;
            case RollerCurrentTuning:
              LoggedTunableNumber.ifChanged(
                  hashCode(),
                  (current) -> world.rollersLeadMotorIO.controlOpenLoopCurrent(Amps.of(current[0])),
                  rollerTuningCurrent);
              break;
            case RollerClosedLoopTuning:
              rollerTuningGains.ifChanged(
                  hashCode(),
                  rollerTuningGains.getMotorIOApplier(
                      world.rollersLeadMotorIO,
                      (PIDGains gains) -> JsonConstants.intakeConstants.rollersPIDGains = gains));
              LoggedTunableNumber.ifChanged(
                  hashCode(), (rpm) -> world.runRollers(RPM.of(rpm[0])), rollerTuningSetpointRPM);
              break;
            default:
              break;
          }
        }
      };

  public static State<IntakeSubsystem> deployedState =
      new State<IntakeSubsystem>("Deployed") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.setTargetPivotAngle(JsonConstants.intakeConstants.intakePositionAngle);
        }
      };

  public static State<IntakeSubsystem> stowedState =
      new State<IntakeSubsystem>("Stowed") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.setTargetPivotAngle(JsonConstants.intakeConstants.stowPositionAngle);
        }
      };

  public static State<IntakeSubsystem> homingWaitForMovementState =
      new State<IntakeSubsystem>("HomingWaitForMovement") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.pivotMotorIO.controlOpenLoopVoltage(JsonConstants.intakeConstants.homingVoltage);

          if (Math.abs(world.pivotInputs.velocityRadiansPerSecond)
              > JsonConstants.intakeConstants.homingMovementThreshold.in(Units.RadiansPerSecond)) {
            finish();
          }
        }
      };

  public static State<IntakeSubsystem> homingWaitForStopMovingState =
      new State<IntakeSubsystem>("HomingWaitForStopMoving") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.pivotMotorIO.controlOpenLoopVoltage(JsonConstants.intakeConstants.homingVoltage);

          if (Math.abs(world.pivotInputs.velocityRadiansPerSecond)
              < JsonConstants.intakeConstants.homingMovementThreshold.in(Units.RadiansPerSecond)) {
            finish();
          }
        }
      };

  public static State<IntakeSubsystem> homingDoneState =
      new State<IntakeSubsystem>("HomingDone") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.pivotMotorIO.controlNeutral();
          world.pivotMotorIO.setCurrentPositionAsZero();
        }
      };

  public static State<IntakeSubsystem> waitForButtonState =
      new State<IntakeSubsystem>("WaitForButton") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          // Do nothing, just wait for the robot to be enabled
        }
      };
}
