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

  private boolean wasHomed = false;
  private boolean isHomed = false;

  // Simple guard: when true we have already started the rainbow on the device and should not
  // resend it each periodic (lumyn's RunOnce(false) will be interrupted if resent every tick).
  private boolean multiFramedAnimationActive = false;

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
    boolean disabled = DriverStation.isDisabled();
    // TODO: replace with homed switch
    isHomed = SmartDashboard.getBoolean("led/isHomed", false);
    // check if isHomed, if not disable all animations and set all strips to be orange
    if (!isHomed) {
      setGroupColor("all", Color.kOrangeRed);
      multiFramedAnimationActive = false;
      wasHomed = false;
      return;
    }
    // main stuff

    if (!disabled) {
      setGroupColor("all", Color.kLime);
    } else {
      playMultiFramedAnimationOnce("all", 6, Animation.RainbowRoll, Color.kBlack);
    }

    wasDisabled = DriverStation.isDisabled();
  }

  public void setHomed(boolean homed) {
    isHomed = homed;
  }

  /**
   * Play the multiframe animation for a group once. This prevents repeatedly resending the same
   * animation every periodic and spamming the command.
   *
   * @param animation animation to play
   * @param group group name
   * @param color optional color (black = none because Lumyn labs FRC color to Lumyn Labs color
   *     logic can't be null. {@link com.lumynlabs.domain.led.LedHandler#FrcColorToAnimationColor
   *     see here})
   * @param delayMillis optional delay in milliseconds (null = none)
   * @param runOnce whether to RunOnce on the device
   */
  private void playMultiFramedAnimationOnce(
      String group, int delayMillis, Animation animation, Color color) {
    if (multiFramedAnimationActive) {
      return;
    }
    led.leds.SetAnimation(animation)
        .WithColor(color)
        .ForGroup(group)
        .WithDelay(Milliseconds.of(delayMillis))
        .RunOnce(false);
    multiFramedAnimationActive = true;
  }

  public void clearGroup(String group) {
    led.leds.SetAnimation(Animation.Fill).WithColor(Color.kBlack).ForGroup(group).RunOnce(true);
    multiFramedAnimationActive = false;
  }

  public void setGroupColor(String group, Color color) {
    led.leds.SetAnimation(Animation.Fill).WithColor(color).ForGroup(group).RunOnce(true);
    multiFramedAnimationActive = false;
  }
}
