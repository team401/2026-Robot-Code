package frc.robot.util.io.dio_switch;

import edu.wpi.first.wpilibj.simulation.DIOSim;

/**
 * DigitalInputIOSim implements DigitalInputIO by extending DigitalInputIOReal and using wpilib's
 * {@link DIOSim} class to update the values that the DigitalInput returns. Use {@link
 * this#setValue(boolean)} to set the value that will be read.
 */
public class DigitalInputIOSim extends DigitalInputIOReal {
  protected final DIOSim dioSim;

  /**
   * Create a new DigitalInputIOSim using a certain DIO channel.
   *
   * <p>For more info on channels, see {@link DigitalInputIOReal#DigitalInputIOReal(int)}.
   *
   * @param channel The DIO channel to use for this simulated DigitalInput.
   */
  public DigitalInputIOSim(int channel) {
    super(channel);
    dioSim = new DIOSim(channel);
    dioSim.setIsInput(true);
  }

  /**
   * Set the value to be returned by the DigitalInput.
   *
   * <p>This value is passed to {@link DIOSim#setValue(boolean)}.
   *
   * @param isOpen Whether the simulated switch is open.
   */
  public void setValue(boolean isOpen) {
    dioSim.setValue(isOpen);
  }
}
