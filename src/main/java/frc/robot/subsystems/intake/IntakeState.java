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
import frc.robot.util.TuningModeHelper;

public class IntakeState {

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

  // The test mode need to be a defined class because we need to be able to
  // easily control when it is created to ensure that the JsonConstants are
  // loaded before we try to create any LoggedTunableNumbers for the test mode
  public static class TestModeState extends State<IntakeSubsystem> {

    private TuningModeHelper<PivotTestMode> pivotTuningModeHelper;
    private TuningModeHelper<RollerTestMode> rollerTuningModeHelper;

    public TestModeState(IntakeSubsystem world) {
      super("TestMode");

      pivotTuningModeHelper = new TuningModeHelper<>(PivotTestMode.class)
        .addStandardTuningModesForMotor(
          PivotTestMode.PivotPhoenixTuning, PivotTestMode.PivotVoltageTuning, PivotTestMode.PivotCurrentTuning,
         "Intake/Pivot/", world.pivotMotorIO)
        .addTuningMode(PivotTestMode.PivotClosedLoopTuning,
            TuningModeHelper.addClosedLoopPositionUnprofiledTuning(
                TuningModeHelper.builder(),
                "Intake/Pivot/",
                JsonConstants.intakeConstants.pivotPIDGains,
                JsonConstants.intakeConstants.stowPositionAngle,
                gains -> JsonConstants.intakeConstants.pivotPIDGains = gains,
                setpoint -> world.setTargetPivotAngle(setpoint),
                world.pivotMotorIO).build());

      rollerTuningModeHelper = new TuningModeHelper<>(RollerTestMode.class)
        .addStandardTuningModesForMotor(
          RollerTestMode.RollerPhoenixTuning, RollerTestMode.RollerVoltageTuning, RollerTestMode.RollerCurrentTuning,
         "Intake/Pivot/", world.pivotMotorIO)
        .addTuningMode(RollerTestMode.RollerClosedLoopTuning,
            TuningModeHelper.addClosedLoopVelocityUnprofiledTuning(
                TuningModeHelper.builder(),
                "Intake/Pivot/",
                JsonConstants.intakeConstants.rollersPIDGains,
                RPM.of(0),
                gains -> JsonConstants.intakeConstants.rollersPIDGains = gains,
                s -> {},
                world.rollersLeadMotorIO, world.rollersFollowerMotorIO).build());
    }

    @Override
    protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {

      pivotTuningModeHelper.runTestMode(world.pivotTestModeManager.getTestMode());
      rollerTuningModeHelper.runTestMode(world.rollerTestModeManager.getTestMode());

      if (!shouldBeInTestMode(world)) {
        finish();
      }
    }
  }

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
