package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.config.ConfigBuilder;
import com.lumynlabs.domain.config.LumynDeviceConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public class LED extends SubsystemBase {
  private Optional<Drive> driveSubsystem;
  // private Optional<Shooter> shooterSubsystem;

  public static ConnectorXAnimate led = new ConnectorXAnimate();
  private boolean ledConnected = led.Connect(USBPort.kUSB1);
  private boolean directLedEnabled = false;

  /* Lumyn Device Config creates a config build for everything that
   * LEDs are going to do (ie. zones, channels, animations)
   */
  private LumynDeviceConfig buildConfig() {
    return new ConfigBuilder()
        .forTeam("401")
        // Channels
        // TODO: these are all made up because design isn't done designing, will update
        .addChannel(1, "leftSide", 60)
        .addStripZone("leftUpper", 20, false)
        .addStripZone("leftMiddle", 20, false)
        .addStripZone("leftLower", 20, false)
        .endChannel()
        .addChannel(2, "rightSide", 60)
        .addStripZone("rightUpper", 20, false)
        .addStripZone("rightMiddle", 20, false)
        .addStripZone("rightLower", 20, false)
        .endChannel()
        // Groups
        .addGroup("all")
        .addZone("leftUpper")
        .addZone("leftMiddle")
        .addZone("leftLower")
        .addZone("rightUpper")
        .addZone("rightMiddle")
        .addZone("rightLower")
        .endGroup()
        .addGroup("leftSide")
        .addZone("leftUpper")
        .addZone("leftMiddle")
        .addZone("leftLower")
        .endGroup()
        .addGroup("rightSide")
        .addZone("rightUpper")
        .addZone("rightMiddle")
        .addZone("rightLower")
        .endGroup()
        .build();
  }

  public LED(Optional<Drive> drive) {
    this.driveSubsystem = drive;
    // this.shooterSubsystem = shooter
    // Put other subsystems here!!
    led.Connect(USBPort.kUSB1);
    led.ApplyConfiguration(buildConfig());
  }

  public void periodic() {
    if (DriverStation.isDisabled()) {
      directLedEnabled = true;

    } else {
      directLedEnabled = false;
      led.leds.SetGroupColor("all", new Color(new Color8Bit(0, 0, 255)));
    }
  }
}
