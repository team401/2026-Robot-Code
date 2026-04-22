package frc.robot.subsystems.homingswitch;

import frc.robot.util.io.dio_switch.DigitalInputIOReplay;

public class HomingSwitchIOReplay extends DigitalInputIOReplay implements HomingSwitchIO {
  @Override
  public void pullupOutput() {
    // Do nothing, the replay will handle this
  }

  @Override
  public void pulldownOutput() {
    // Do nothing, the replay will handle this
  }
}
