package frc.robot.subsystems;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.config.ConfigBuilder;
import com.lumynlabs.domain.config.LumynDeviceConfig;
import com.lumynlabs.domain.led.Animation;
import edu.wpi.first.units.Units;
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

  /* Lumyn Device Config creates a config build for everything that
   * LEDs are going to do (ie. zones, channels, animations)
   */
  private LumynDeviceConfig buildConfig() {
    return new ConfigBuilder()
        .forTeam("401")
        // Channels
        // TODO: these are all made up because design isn't done designing, will update
        .addChannel(1, "leftBallTower", 159)
        .addStripZone("leftUpperThirdBallTower", 53, false)
        .addStripZone("leftMiddleThirdBallTower", 53, false)
        .addStripZone("leftLowerThirdBallTower", 53, false)
        .endChannel()
        .addChannel(2, "rightBallTower", 159)
        .addStripZone("rightUpperThirdBallTower", 53, true)
        .addStripZone("rightMiddleThirdBallTower", 53, true)
        .addStripZone("leftLowerThirdBallTower", 53, true)
        .endChannel()
        .addGroup("all")
        .addZone("leftUpperThirdBallTower")
        .addZone("leftMiddleThirdBallTower")
        .addZone("leftLowerThirdBallTower")
        .addZone("rightUpperThirdBallTower")
        .addZone("rightMiddleThirdBallTower")
        .addZone("rightLowerThirdBallTower")
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
    if (!DriverStation.isDisabled()) {
      led.leds.SetAnimation(Animation.RainbowCycle)
          .ForGroup("all") // group
          .WithDelay(Units.Milliseconds.of(40)) // time between animation frames
          .Reverse(false) // reversed or not
          .RunOnce(false); // looped or not
    }
    else {
      led.leds.SetGroupColor("all", new Color(new Color8Bit(0, 255, 0)));
    }
  }
}
