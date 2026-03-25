package frc.robot.subsystems.intake;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import frc.robot.constants.JsonConstants;

/** Contains states for the intake rollers. */
public class IntakeRollerState {
  private IntakeRollerState() {}

  public static State<IntakeSubsystem> stopRollersState =
      new State<IntakeSubsystem>("StopRollers") {
        @Override
        public void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.rollersLeadMotorIO.controlNeutral();
        }
      };

  public static State<IntakeSubsystem> runRollersState =
      new State<IntakeSubsystem>("RunRollers") {
        @Override
        public void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.rollersLeadMotorIO.controlToVelocityUnprofiled(world.requestedRollerSpeed);
        }
      };

  public static State<IntakeSubsystem> dejamState =
      new State<IntakeSubsystem>("Dejam") {
        @Override
        public void periodic(StateMachine<IntakeSubsystem> stateMachine, IntakeSubsystem world) {
          world.rollersLeadMotorIO.controlOpenLoopVoltage(
              JsonConstants.intakeConstants.rollerDejamVoltage);
        }
      };
}
