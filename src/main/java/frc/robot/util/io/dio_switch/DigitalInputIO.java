package frc.robot.util.io.dio_switch;

import org.littletonrobotics.junction.AutoLog;

/**
 * A DigitalInputIO wraps a WPILib DigitalInput in a hardware-abstraction such that it can be
 * properly used in real life, simulation, and replay in accordance with the AdvantageKit
 * architecture.
 */
public interface DigitalInputIO {
  /** Inputs from a DigitalInput. */
  @AutoLog
  public static class DigitalInputInputs {
    /** Whether or not the circuit was open last time these inputs were updated */
    public boolean isOpen = false;
  }

  /**
   * Take a set of inputs and update them with the latest values from hardware or simulation.
   *
   * @param inputs The DigitalInputInputs object to update
   */
  public void updateInputs(DigitalInputInputs inputs);
}
