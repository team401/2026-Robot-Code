package frc.robot.util.io.dio_switch;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import coppercore.wpilib_interface.CTREUtil;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The DigitalInputIOCANdi class implements the DigitalInputIO interface using a CANdi to read a
 * digital input via the CAN bus.
 */
public class DigitalInputIOCANdi implements DigitalInputIO {
  public enum CANdiSignal {
    S1,
    S2
  }

  protected final String deviceName;

  protected final CANdi candi;
  protected final StatusSignal<Boolean> closedSignal;

  protected final Alert configFailedToApplyAlert;

  protected final Alert disconnectedAlert;

  /**
   * The number of times updateInputs has been called. When less than 3, the inputs should be
   * ignored (not updated). The first update after connection provides a stale "false" value for
   * isClosed, resulting in the homing switch being stuck "pressed" during startup.
   */
  protected int readCount = 0;

  public DigitalInputIOCANdi(CANDeviceID id, CANdiConfiguration candiConfig, CANdiSignal signal) {
    this.candi = new CANdi(id.id(), id.canbus());

    this.deviceName = "CANdi " + id;

    String configFailedToApplyMessage = deviceName + " failed to apply configs.";

    this.configFailedToApplyAlert = new Alert(configFailedToApplyMessage, AlertType.kError);

    String disconnectedMessage = deviceName + " disconnected.";

    this.disconnectedAlert = new Alert(disconnectedMessage, AlertType.kError);

    CTREUtil.tryUntilOk(
        () -> this.candi.getConfigurator().apply(candiConfig),
        id,
        code -> configFailedToApplyAlert.set(true));

    switch (signal) {
      case S1 -> this.closedSignal = candi.getS1Closed();
      case S2 -> this.closedSignal = candi.getS2Closed();
      default -> throw new UnsupportedOperationException("Unknown CANdi signal " + signal);
    }

    CTREUtil.tryUntilOk(() -> closedSignal.setUpdateFrequency(Hertz.of(50)), id, code -> {});
    CTREUtil.tryUntilOk(() -> candi.optimizeBusUtilization(), id, code -> {});
  }

  public void updateInputs(DigitalInputInputs inputs) {
    StatusCode code = BaseStatusSignal.refreshAll(closedSignal);

    disconnectedAlert.set(!code.isOK());

    if (code.isError()) {
      DriverStation.reportError(deviceName + ": Failed to refresh status signals: " + code, false);
    } else if (code.isWarning()) {
      DriverStation.reportWarning(
          deviceName + ": Warning while refreshing status signals: " + code, false);
    }

    inputs.connected = code.isOK();
    if (inputs.connected && closedSignal.hasUpdated()) {
      if (readCount < 3) {
        // Only increment readCount inside here to prevent an eventual overflow issue during very
        // long runtimes
        // Doing the math, it would take about 1.5 years of continuous runtime at 50hz to cause this
        // to occur, but in the spirit of doing things the right way this check should exist anyway.
        readCount++;
        return;
      }
      inputs.isOpen = !closedSignal.getValue();
    }
  }
}
