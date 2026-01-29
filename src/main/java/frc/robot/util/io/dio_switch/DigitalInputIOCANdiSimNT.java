package frc.robot.util.io.dio_switch;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.sim.CANdiSimState;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import frc.robot.util.ExtraCTREUtil.DigitalSignalCloseState;
import frc.robot.util.io.dio_switch.DigitalInputIO.DigitalInputInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class DigitalInputIOCANdiSimNT extends DigitalInputIOCANdi {
  protected final CANdiSimState candiSimState;
  protected final CANdiSignal signal;
  protected final LoggedNetworkBoolean simValue;

  protected final DigitalSignalCloseState closeState;

  public DigitalInputIOCANdiSimNT(
      CANDeviceID id,
      CANdiConfiguration candiConfig,
      CANdiSignal signal,
      String ntPath,
      boolean defaultIsOpenValue) {
    super(id, candiConfig, signal);
    this.candiSimState = candi.getSimState();
    this.signal = signal;

    switch (signal) {
      case S1 ->
          this.closeState = DigitalSignalCloseState.from(candiConfig.DigitalInputs.S1CloseState);
      case S2 ->
          this.closeState = DigitalSignalCloseState.from(candiConfig.DigitalInputs.S2CloseState);
      default -> throw new UnsupportedOperationException("Unknown CANdiSignal " + signal);
    }

    this.simValue = new LoggedNetworkBoolean(ntPath, defaultIsOpenValue);
  }

  @Override
  public void updateInputs(DigitalInputInputs inputs) {
    boolean openState = simValue.get();

    switch (signal) {
      case S1 -> candiSimState.setS1State(closeState.getS1FromOpenState(openState));
      case S2 -> candiSimState.setS2State(closeState.getS2FromOpenState(openState));
    }

    super.updateInputs(inputs);
  }
}
