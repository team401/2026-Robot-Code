package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.config.ConfigBuilder;
import com.lumynlabs.domain.config.LumynDeviceConfig;
import com.lumynlabs.domain.led.Animation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CoordinationLayer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class LED extends SubsystemBase {
  private Optional<Drive> driveSubsystem;
  private Optional<HoodSubsystem> hoodSubsystem;
  private Optional<HopperSubsystem> hopperSubsystem;
  private Optional<ShooterSubsystem> shooterSubsystem;
  private Optional<TurretSubsystem> turretSubsystem;
  private Optional<CoordinationLayer> coordinationLayer;

  public static ConnectorXAnimate led = new ConnectorXAnimate();
  private boolean ledConnected = led.Connect(USBPort.kUSB1);
  private boolean directLedEnabled = false;

  private boolean wasDisabled = false;
  private boolean disabled = DriverStation.isDisabled();

  private boolean wasHomed = false;
  private boolean isHomed = false;
  private final LoggedNetworkBoolean isHomedSpoof = new LoggedNetworkBoolean("LED/isHomed");

  /* Lumyn Device Config creates a config build for everything that
   * LEDs are going to do (ie. zones, channels, animations)
   */
  private LumynDeviceConfig buildConfig() {
    return new ConfigBuilder()
        .forTeam("401")
        // Channels

        .addChannel(1, "backStrip", 94)
        .addStripZone("leftBack", 47, false)
        .addStripZone("rightBack", 47, false)
        .endChannel()
        .addChannel(2, "frontStrip", 102)
        .addStripZone("leftFront", 51, false)
        .addStripZone("rightFront", 51, false)
        .endChannel()
        // Group
        .addGroup("backStrip")
        .addZone("leftBack")
        .addZone("rightBack")
        .endGroup()
        .addGroup("frontStrip")
        .addZone("leftFront")
        .addZone("rightFront")
        .endGroup()
        .addGroup("all")
        .addZone("leftBack")
        .addZone("rightBack")
        .addZone("leftFront")
        .addZone("rightFront")
        .endGroup()
        .build();
  }

  public LED(
      Optional<Drive> drive,
      Optional<HoodSubsystem> hood,
      Optional<HopperSubsystem> hopper,
      Optional<ShooterSubsystem> shooter,
      Optional<TurretSubsystem> turret,
      Optional<CoordinationLayer> coordinationLayer) {
    this.driveSubsystem = drive;
    this.hoodSubsystem = hood;
    this.hopperSubsystem = hopper;
    this.shooterSubsystem = shooter;
    this.turretSubsystem = turret;
    this.coordinationLayer = coordinationLayer;

    led.Connect(USBPort.kUSB1);
    led.ApplyConfiguration(buildConfig());
    clearGroup("all");
    setGroupColor("all", Color.kYellow); // see time between init and periodic

    SmartDashboard.putBoolean("led/isHomed", isHomed);
  }

  public void periodic() {
    isHomed = SmartDashboard.getBoolean("led/isHomed", false);
    // check if homed switch has been triggered
    if (!isHomed) {
      setGroupColor("all", Color.kOrangeRed);
      wasHomed = false;
      return;
    } else if (isHomed && !wasHomed) {
      if (disabled && !wasDisabled) {
        led.leds.SetAnimation(Animation.RainbowRoll)
            .ForGroup("all")
            .WithDelay(Milliseconds.of(6))
            .RunOnce(false);
      }
    }

    if (!disabled && wasDisabled) {
      led.leds.SetAnimation(Animation.Fill).ForGroup("all").WithColor(Color.kBlack).RunOnce(true);

      led.leds.SetColor("rightFront", Color.kPink);
    }

    wasDisabled = disabled;
    wasHomed = isHomed;
  }

  public void setHomed(boolean homed) {
    isHomed = homed;
  }

  public void clearGroup(String group) {
    led.leds.SetAnimation(Animation.Fill).WithColor(Color.kBlack).ForGroup(group).RunOnce(true);
  }

  public void setGroupColor(String group, Color color) {
    led.leds.SetAnimation(Animation.Fill).WithColor(color).ForGroup(group).RunOnce(true);
  }
}
