package frc.robot.subsystems.homingswitch;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import frc.robot.util.ServiceThread;
import frc.robot.util.io.dio_switch.DigitalInputIOCANdi;

public class HomingSwitchIOReal extends DigitalInputIOCANdi implements HomingSwitchIO {
  protected final CANdiSignal outputSignal;
  protected final CANdiConfiguration candiConfig;

  public HomingSwitchIOReal(
      CANDeviceID id,
      CANdiConfiguration candiConfig,
      CANdiSignal inputSignal,
      CANdiSignal outputSignal) {
    super(id, candiConfig, inputSignal);
    this.outputSignal = outputSignal;
    this.candiConfig = candiConfig;
  }

  @Override
  public void pullupOutput() {
    switch (outputSignal) {
      case S1:
        candiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
        break;
      case S2:
        candiConfig.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
        break;
    }

    applyConfig();
  }

  @Override
  public void pulldownOutput() {
    switch (outputSignal) {
      case S1:
        candiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.PullLow;
        break;
      case S2:
        candiConfig.DigitalInputs.S2FloatState = S2FloatStateValue.PullLow;
        break;
    }

    applyConfig();
  }

  private void applyConfig() {
    ServiceThread.defaultServiceThread.queueCommand(
        () -> {
          this.candi.getConfigurator().apply(candiConfig);
        });
  }
}
