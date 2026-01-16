package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;

import java.util.Optional;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.config.ConfigBuilder;
import com.lumynlabs.domain.config.LumynDeviceConfig;

import frc.robot.subsystems.drive.Drive;

public class LED {
    private Optional<Drive> driveSubsystem;

    public static ConnectorXAnimate led = new ConnectorXAnimate();
    private boolean ledConnected = led.Connect(USBPort.kUSB1);
  /* Lumyn Device Config creates a config build for everything that
   * LEDs are going to do (ie. zones, channels, animations)
   */
  
    public LED(Optional<Drive> drive) {
    this.driveSubsystem = drive;
    
  }

    public LED(com.google.common.base.Optional<Drive> drive) {
        //TODO Auto-generated constructor stub
    }

}
