package frc.robot.subsystems;

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
  // Track whether we were disabled in the previous periodic call so we only send the
  // animation command once when entering the disabled state (prevents spamming the device
  // with the same animation every loop).
  private boolean wasDisabled = false;
  private boolean isHomed = false;

  /* Lumyn Device Config creates a config build for everything that
   * LEDs are going to do (ie. zones, channels, animations)
   */
  private LumynDeviceConfig buildConfig() {
    return new ConfigBuilder()
        .forTeam("401")
        // Channels
        // TODO: these are all made up because design isn't done designing, will update
        .addChannel(1, "1", 160)
        .addStripZone("1", 160, false)
        .endChannel()
        // Group
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
    led.leds.SetAnimation(Animation.Fill)
        .ForGroup("1")
        .WithColor(Color.kYellow)
        .RunOnce(true); // see time between init and periodic

    SmartDashboard.putBoolean("led/isHomed", isHomed);
  }

  public void periodic() {
    isHomed = SmartDashboard.getBoolean("led/isHomed", false);
    if (isHomed == false) {
      led.leds.SetAnimation(Animation.Fill).ForZone("1").WithColor(Color.kMagenta).RunOnce(true);
      return;
    }
    boolean disabled = DriverStation.isDisabled();

    // When we first enter the disabled state, start the looping rainbow animation once. The
    // animation itself will run continuously on the LED controller, so we don't need to keep
    // re-sending the command every periodic call (which can spam the device).
    if (disabled && !wasDisabled) {
      led.leds.SetAnimation(Animation.RainbowCycle)
          .ForZone("1")
          .WithDelay(Milliseconds.of(10))
          .RunOnce(false);
    }

    // When we transition back to enabled, set a single color update (only once on transition)
    // to avoid spamming the controller every loop.
    if (!disabled && wasDisabled) {
      led.leds.SetAnimation(Animation.Fill).ForZone("1").WithColor(Color.kBlack).RunOnce(true);
      // led.leds.SetGroupColor("1", Color.kPurple);
      led.leds.SetColor("1", Color.kPink);
    }

    // Update state for next periodic call
    wasDisabled = disabled;
  }

  public void setHomed(boolean homed) {
    isHomed = homed;
  }
}
