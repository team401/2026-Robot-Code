package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.units.Units;
import frc.robot.constants.JsonConstants;
import frc.robot.util.TuningModeHelper;
import frc.robot.util.TuningModeHelper.ControlMode;
import frc.robot.util.TuningModeHelper.MotorTuningMode;
import frc.robot.util.TuningModeHelper.TunableMotor;
import frc.robot.util.TuningModeHelper.TunableMotorConfiguration;

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

    private TuningModeHelper<PivotTestMode> pivotTuningModeHelper;
    private TuningModeHelper<RollerTestMode> rollerTuningModeHelper;

    public TestModeState(IntakeSubsystem world) {
      super("TestMode");

      TunableMotor pivotMotor =
          TunableMotorConfiguration.defaultConfiguration()
              .withPositionTuning()
              .withDefaultPIDGains(JsonConstants.intakeConstants.pivotPIDGains)
              .onPIDGainsChanged(gains -> JsonConstants.intakeConstants.pivotPIDGains = gains)
              .withTunableAngleUnit(Degrees)
              .build("Intake/PivotMotorTuning", world.pivotMotorIO);
      TunableMotor rollerMotors =
          TunableMotorConfiguration.defaultConfiguration()
              .withVelocityTuning()
              .withDefaultPIDGains(JsonConstants.intakeConstants.rollersPIDGains)
              .onPIDGainsChanged(gains -> JsonConstants.intakeConstants.rollersPIDGains = gains)
              .withTunableAngularVelocityUnit(RPM)
              .build(
                  "Intake/RollersMotorTuning",
                  world.rollersLeadMotorIO,
                  world.rollersFollowerMotorIO);

      pivotTuningModeHelper =
          new TuningModeHelper<>(PivotTestMode.class)
              .addMotorTuningModes(
                  pivotMotor,
                  MotorTuningMode.of(PivotTestMode.None, ControlMode.NONE),
                  MotorTuningMode.of(PivotTestMode.PivotPhoenixTuning, ControlMode.PHOENIX_TUNING),
                  MotorTuningMode.of(
                      PivotTestMode.PivotVoltageTuning, ControlMode.OPEN_LOOP_VOLTAGE),
                  MotorTuningMode.of(
                      PivotTestMode.PivotCurrentTuning, ControlMode.OPEN_LOOP_CURRENT),
                  MotorTuningMode.of(PivotTestMode.PivotClosedLoopTuning, ControlMode.CLOSED_LOOP));

      rollerTuningModeHelper =
          new TuningModeHelper<>(RollerTestMode.class)
              .addMotorTuningModes(
                  rollerMotors,
                  MotorTuningMode.of(RollerTestMode.None, ControlMode.NONE),
                  MotorTuningMode.of(
                      RollerTestMode.RollerPhoenixTuning, ControlMode.PHOENIX_TUNING),
                  MotorTuningMode.of(
                      RollerTestMode.RollerVoltageTuning, ControlMode.OPEN_LOOP_VOLTAGE),
                  MotorTuningMode.of(
                      RollerTestMode.RollerCurrentTuning, ControlMode.OPEN_LOOP_CURRENT),
                  MotorTuningMode.of(
                      RollerTestMode.RollerClosedLoopTuning, ControlMode.CLOSED_LOOP),
                  MotorTuningMode.of(RollerTestMode.RollerSpeedTuning, ControlMode.NEUTRAL_MODE));
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

      pivotTuningModeHelper.testPeriodic(world.pivotTestModeManager.getTestMode());
      rollerTuningModeHelper.testPeriodic(world.rollerTestModeManager.getTestMode());

      if (!shouldBeInTestMode(world)) {
        // Ensure motors are neutral when exiting test mode
        world.pivotMotorIO.controlNeutral();
        world.stopRollers();
        finish();
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
