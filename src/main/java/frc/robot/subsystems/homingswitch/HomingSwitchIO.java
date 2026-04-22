package frc.robot.subsystems.homingswitch;

import frc.robot.util.io.dio_switch.DigitalInputIO;

public interface HomingSwitchIO extends DigitalInputIO {
  public void pullupOutput();

  public void pulldownOutput();
}
