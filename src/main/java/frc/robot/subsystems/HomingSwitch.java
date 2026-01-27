package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import frc.robot.DependencyOrderedExecutor;
import frc.robot.DependencyOrderedExecutor.ActionKey;
import frc.robot.util.io.dio_switch.DigitalInputIO;
import frc.robot.util.io.dio_switch.DigitalInputInputsAutoLogged;

public class HomingSwitch {
   private final DigitalInputIO io; 
   private final DigitalInputInputsAutoLogged inputs = new DigitalInputInputsAutoLogged(); 

   public static final ActionKey UPDATE_INPUTS = new ActionKey("HomingSwitch::updateInputs");

   public HomingSwitch(DependencyOrderedExecutor dependencyOrderedExecutor, DigitalInputIO io) {
    this.io = io;

    dependencyOrderedExecutor.registerAction(UPDATE_INPUTS, this::updateInputs);
   }

   public void updateInputs() {
        io.updateInputs(inputs);

        Logger.processInputs("HomingSwitch/inputs", inputs);
   }

   /**
    * Return `true` if the homing switch is pressed, `false` otherwise
    * @return `true` if the homing switch is pressed, `false` otherwise
    */
   public boolean isHomingSwitchPressed() {
        // TODO: Verify that "open" means the switch is pressed and not the opposite
        return inputs.isOpen; 
   }
}
