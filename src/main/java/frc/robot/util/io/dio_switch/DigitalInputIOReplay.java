package frc.robot.util.io.dio_switch;

/**
 * DigitalInputIOReplay allows DigitalInputIO to be used in replay by implementing methods with
 * no-op implementations.
 */
public class DigitalInputIOReplay implements DigitalInputIO {
  @Override
  public void updateInputs(DigitalInputInputs inputs) {}
}
