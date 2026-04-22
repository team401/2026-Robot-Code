package frc.robot.subsystems.homingswitch;

import frc.robot.DependencyOrderedExecutor;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.util.io.dio_switch.DigitalInputInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class HomingSwitch {
  private final HomingSwitchIO io;
  private final DigitalInputInputsAutoLogged inputs = new DigitalInputInputsAutoLogged();

  public static final ActionKey UPDATE_INPUTS = new ActionKey("HomingSwitch::updateInputs");

  enum HomingState {
    WAIT_FOR_STARTUP,
    WAIT_FOR_HOME,
    HOMED
  }

  private HomingState homingState = HomingState.WAIT_FOR_STARTUP;

  public HomingSwitch(DependencyOrderedExecutor dependencyOrderedExecutor, HomingSwitchIO io) {
    this.io = io;

    dependencyOrderedExecutor.registerAction(UPDATE_INPUTS, this::updateInputs);
  }

  public void updateInputs() {
    io.updateInputs(inputs);

    Logger.processInputs("HomingSwitch/inputs", inputs);

    switch (homingState) {
      case WAIT_FOR_STARTUP:
        // If periodic is running, we have started up
        homingState = HomingState.WAIT_FOR_HOME;
        io.pullupOutput();
        break;
      case WAIT_FOR_HOME:
        if (isHomingSwitchPressed()) {
          homingState = HomingState.HOMED;
          io.pulldownOutput();
        }
        homingState = HomingState.HOMED;
        break;
      case HOMED:
        // Do nothing
        break;
    }
  }

  /**
   * Return `true` if the homing switch is pressed, `false` otherwise
   *
   * @return `true` if the homing switch is pressed, `false` otherwise
   */
  public boolean isHomingSwitchPressed() {
    // TODO: Verify that "open" means the switch is pressed and not the opposite
    return inputs.isOpen;
  }
}
