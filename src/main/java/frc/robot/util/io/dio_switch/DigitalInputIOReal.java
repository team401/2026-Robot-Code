package frc.robot.util.io.dio_switch;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * DigitalInputIOReal implements DigitalInputIO to update a value based on a physical DigitalInput
 * on the RoboRIO.
 */
public class DigitalInputIOReal implements DigitalInputIO {
  protected final DigitalInput digitalInput;

  /**
   * Construct a new Real DigitalInputIO on a specific channel.
   *
   * @param channel An int representing the DIO channel. Ports 0-9 are the physical DIO ports on the
   *     RIO, while 10-25 are available through the <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/frc-glossary.html#term-MXP">MXP
   *     port</a>.
   */
  public DigitalInputIOReal(int channel) {
    this.digitalInput = new DigitalInput(channel);
  }

  @Override
  public void updateInputs(DigitalInputInputs inputs) {
    inputs.isOpen = digitalInput.get();
  }
}
