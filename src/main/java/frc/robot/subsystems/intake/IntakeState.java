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

public class IntakeState {

  // ### Test Mode State

  public static boolean shouldBeInTestMode(IntakeSubsystem world) {
    // If the pivot is in test mode, we should be in test mode
    // because every pivot test mode directly controls the pivot motor
    if (world.pivotTestModeManager.isInTestMode()) {
      return true;
    }
    // Because Roller Speed Tuning is not really a test mode, we need to special case it here
    // Because it modifies the roller speed setpoint rather than directly controlling the motor
    // And to actually test it we need the pivot to be in normal operation if we are not
    // Manually controlling the pivot with a test mode
    if (world.rollerTestModeManager.getTestMode() == RollerTestMode.RollerSpeedTuning) {
      return false;
    }
    return world.rollerTestModeManager.isInTestMode();
  }

  public static State<IntakeSubsystem> testModeState = null;

  // The test mode needs to be a defined class because we need to be able to
  // easily control when it is created to ensure that the JsonConstants are
  // loaded before we try to create any LoggedTunableNumbers for the test mode
  public static class TestModeState extends State<IntakeSubsystem> {

    private LoggedTunableNumber pivotTuningVoltage;
    private LoggedTunableNumber rollerTuningVoltage;
    private LoggedTunableNumber pivotTuningCurrent;
    private LoggedTunableNumber rollerTuningCurrent;
    private LoggedTunableNumber pivotTuningSetpointDegrees;
    private LoggedTunableNumber rollerTuningSetpointRPM;

    private LoggedTunablePIDGains pivotTuningGains;
    private LoggedTunablePIDGains rollerTuningGains;

    public TestModeState(IntakeSubsystem world) {
      super("TestMode");

      pivotTuningVoltage =
          new LoggedTunableNumber(
              "Intake/PivotVoltageTuning",
              0.0); // Default to 0 volts for safety, since this directly controls the motor
      rollerTuningVoltage =
          new LoggedTunableNumber(
              "Intake/RollerVoltageTuning",
              0.0); // Default to 0 volts for safety, since this directly controls the motor
      pivotTuningCurrent =
          new LoggedTunableNumber(
              "Intake/PivotCurrentTuning",
              0.0); // Default to 0 amps for safety, since this directly controls the motor
      rollerTuningCurrent =
          new LoggedTunableNumber(
              "Intake/RollerCurrentTuning",
              0.0); // Default to 0 amps for safety, since this directly controls the motor
      pivotTuningSetpointDegrees =
          new LoggedTunableNumber(
              "Intake/PivotSetpointDegrees",
              JsonConstants.intakeConstants.stowPositionAngle.in(
                  Units
                      .Degrees)); // Assumes the stow position is the safest default position, since
      // this directly controls the motor
      rollerTuningSetpointRPM =
          new LoggedTunableNumber(
              "Intake/RollerSetpointRPM",
              0.0); // Default to 0 RPM for safety, since this directly controls the motor

      pivotTuningGains =
          new LoggedTunablePIDGains(
              "Intake/PivotPIDGains", JsonConstants.intakeConstants.pivotPIDGains);
      rollerTuningGains =
          new LoggedTunablePIDGains(
              "Intake/RollerPIDGains", JsonConstants.intakeConstants.rollersPIDGains);
    }

    @Override
    public void onEntry(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
      // When we enter test mode, we want to make sure that all motors are neutral
      world.pivotMotorIO.controlNeutral();
      world.stopRollers();
    }

    @Override
    protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {

      world.zeroPositionIfBelowZero();

      pivotTestPeriodic(world.pivotTestModeManager.getTestMode(), world);
      rollerTestPeriodic(world.rollerTestModeManager.getTestMode(), world);

      if (!shouldBeInTestMode(world)) {
        // Ensure motors are neutral when exiting test mode
        world.pivotMotorIO.controlNeutral();
        world.stopRollers();
        finish();
      }
    }

    public void pivotTestPeriodic(PivotTestMode testMode, IntakeSubsystem world) {
      switch (testMode) {
        case PivotVoltageTuning:
          world.pivotMotorIO.controlOpenLoopVoltage(Volts.of(pivotTuningVoltage.getAsDouble()));
          break;
        case PivotCurrentTuning:
          world.pivotMotorIO.controlOpenLoopCurrent(Amps.of(pivotTuningCurrent.getAsDouble()));
          break;
        case PivotClosedLoopTuning:
          pivotTuningGains.ifChanged(
              hashCode(),
              pivotTuningGains
                  .getMotorIOApplier(world.pivotMotorIO)
                  .chain(gains -> JsonConstants.intakeConstants.pivotPIDGains = gains));

          world.pivotMotorIO.controlToPositionUnprofiled(
              Degrees.of(pivotTuningSetpointDegrees.getAsDouble()));
          break;
        default:
          world.pivotMotorIO.controlNeutral();
          break;
      }
    }

    public void rollerTestPeriodic(RollerTestMode testMode, IntakeSubsystem world) {
      switch (testMode) {
        case RollerVoltageTuning:
          System.out.println(
              "Running roller voltage tuning With voltage: " + rollerTuningVoltage.getAsDouble());
          world.rollersLeadMotorIO.controlOpenLoopVoltage(
              Volts.of(rollerTuningVoltage.getAsDouble()));
          break;
        case RollerCurrentTuning:
          System.out.println(
              "Running roller current tuning With current: " + rollerTuningCurrent.getAsDouble());
          world.rollersLeadMotorIO.controlOpenLoopCurrent(
              Amps.of(rollerTuningCurrent.getAsDouble()));
          break;
        case RollerClosedLoopTuning:
          rollerTuningGains.ifChanged(
              hashCode(),
              rollerTuningGains
                  .getMotorIOAppliers(world.rollersLeadMotorIO, world.rollersFollowerMotorIO)
                  .chain(gains -> JsonConstants.intakeConstants.rollersPIDGains = gains));

          world.runRollers(RPM.of(rollerTuningSetpointRPM.getAsDouble()));
          break;
        case None:
          // Ensure rollers are stopped when not in a roller test mode, just in case
          world.stopRollers();
          break;
        default:
          break;
      }
    }
  }

  // ### Homing States

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
        public void onEntry(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          // Ensure the motor is stopped when we enter the done state, just in case
          world.pivotMotorIO.controlNeutral();
        }

        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.pivotMotorIO.setCurrentPositionAsZero();
          finish();
        }
      };

  // ### Manual Homing State

  public static State<IntakeSubsystem> waitForButtonState =
      new State<IntakeSubsystem>("WaitForButton") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          if (world.getDependencies().isHomingSwitchPressed()) {
            world.pivotMotorIO.setCurrentPositionAsZero();
            finish();
          }
        }
      };

  // ### Normal Operation States
  public static State<IntakeSubsystem> controlToPositionState =
      new State<IntakeSubsystem>("ControlToPosition") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.zeroPositionIfBelowZero();
          world.controlToTargetPivotAngle();
        }
      };
}
