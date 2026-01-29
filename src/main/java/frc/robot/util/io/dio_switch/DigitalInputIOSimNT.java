package frc.robot.util.io.dio_switch;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * DigitalInputIOSimNT extends DigitalInputIOSim to update the simulated value of the DigitalInput
 * using a LoggedNetworkBoolean. This makes it possible to use Elastic or AdvantageScope to mock the
 * value of the switch in simulation.
 *
 * @apiNote {@link DigitalInputIOSimNT#setValue(boolean)} should not be called, as value passed into
 *     it will be overridden in updateInputs anyway.
 */
public class DigitalInputIOSimNT extends DigitalInputIOSim {
  protected final LoggedNetworkBoolean simValue;

  /**
   * Construct a new DigitalInputIOSimNT.
   *
   * @param ntPath The network tables path used to publish and read the switch value
   * @param channel The DIO channel. See {@link DigitalInputIOReal#DigitalInputIOReal(int)}
   * @param initiallyIsOpen Whether the default value of the switch should default to being open.
   *     True if isOpen should initially be published as true, false if not.
   */
  public DigitalInputIOSimNT(String ntPath, int channel, boolean initiallyIsOpen) {
    super(channel);
    simValue = new LoggedNetworkBoolean(ntPath, initiallyIsOpen);
  }

  @Override
  public void updateInputs(DigitalInputInputs inputs) {
    setValue(simValue.get());
    super.updateInputs(inputs);
  }
}
