package frc.robot.subsystems.intake;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.units.Units;
import frc.robot.constants.JsonConstants;

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

    public static State<IntakeSubsystem> testModeState = new State<IntakeSubsystem>("TestMode") {

        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
            
            pivotTestPeriodic(world.pivotTestModeManager.getTestMode());
            rollerTestPeriodic(world.rollerTestModeManager.getTestMode());

            if (!shouldBeInTestMode(world)) {
                finish();
            }
        }

        public void pivotTestPeriodic(PivotTestMode testMode) {
            switch (testMode) {
                case PivotVoltageTuning:
                    // Implement voltage tuning logic here
                    break;
                case PivotCurrentTuning:
                    // Implement current tuning logic here
                    break;
                case PivotClosedLoopTuning:
                    // Implement closed-loop tuning logic here
                    break;
                default:
                    break;
            }
        }

        public void rollerTestPeriodic(RollerTestMode testMode) {
            switch (testMode) {
                case RollerVoltageTuning:
                    // Implement voltage tuning logic here
                    break;
                case RollerCurrentTuning:
                    // Implement current tuning logic here
                    break;
                case RollerClosedLoopTuning:
                    // Implement closed-loop tuning logic here
                    break;
                default:
                    break;
            }
        }

    };

    public static State<IntakeSubsystem> deployedState = new State<IntakeSubsystem>("Deployed") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
            world.setTargetPivotAngle(JsonConstants.intakeConstants.intakePositionAngle);
        }
    };

    public static State<IntakeSubsystem> stowedState = new State<IntakeSubsystem>("Stowed") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
            world.setTargetPivotAngle(JsonConstants.intakeConstants.stowPositionAngle);
        }
    };

    public static State<IntakeSubsystem> homingWaitForMovementState = new State<IntakeSubsystem>("HomingWaitForMovement") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
            world.pivotMotorIO.controlOpenLoopVoltage(JsonConstants.intakeConstants.homingVoltage);
            
            if (Math.abs(world.pivotInputs.velocityRadiansPerSecond) > JsonConstants.intakeConstants.homingMovementThreshold.in(Units.RadiansPerSecond)) {
                finish();
            }
        }
    };

    public static State<IntakeSubsystem> homingWaitForStopMovingState = new State<IntakeSubsystem>("HomingWaitForStopMoving") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
            world.pivotMotorIO.controlOpenLoopVoltage(JsonConstants.intakeConstants.homingVoltage);

            if (Math.abs(world.pivotInputs.velocityRadiansPerSecond) < JsonConstants.intakeConstants.homingMovementThreshold.in(Units.RadiansPerSecond)) {
                finish();
            }
        }
    };

    public static State<IntakeSubsystem> homingDoneState = new State<IntakeSubsystem>("HomingDone") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
            world.pivotMotorIO.controlNeutral();
            world.pivotMotorIO.setCurrentPositionAsZero();
        }
    };

    public static State<IntakeSubsystem> waitForButtonState = new State<IntakeSubsystem>("WaitForButton") {
        @Override
        protected void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
            // Do nothing, just wait for the robot to be enabled
        }
    };

}
