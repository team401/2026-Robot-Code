package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.config.ConfigBuilder;
import com.lumynlabs.domain.config.LumynDeviceConfig;
import com.lumynlabs.domain.led.Animation;
import com.lumynlabs.domain.led.DirectLED;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CoordinationLayer;
import frc.robot.coordination.MatchState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class LED extends SubsystemBase {
  private Optional<Drive> driveSubsystem;
  private Optional<HoodSubsystem> hoodSubsystem;
  private Optional<HopperSubsystem> hopperSubsystem;
  private Optional<ShooterSubsystem> shooterSubsystem;
  private Optional<TurretSubsystem> turretSubsystem;
  private Optional<CoordinationLayer> coordinationLayer;
  private MatchState matchState;

  public static ConnectorXAnimate led = new ConnectorXAnimate();

  private boolean wasDisabled = false;
  private boolean multiFramedAnimationActive = false;
  private boolean wasHomed = false;
  private boolean isHomed = false;
  // colors
  private Color mistyNeonTeal = new Color(93, 231, 223);
  public static LEDPattern rainbowPattern =
      LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(40)).reversed();
  // direct leds stuff
  private DirectLED leftBackLED;
  private DirectLED rightBackLED;
  private AddressableLEDBuffer directLEDBackBuffer =
      new AddressableLEDBuffer(47); // length of both back sides
  private Distance ledSpacing = Meters.of(1.0 / 160.0);
  // time left in shift mask
  private LEDPattern matchShiftProgressPattern =
      rainbowPattern.mask(
          LEDPattern.progressMaskLayer(
              () ->
                  matchState.getTimeLeftInCurrentShift() / matchState.getMaxTimeInCurrentShift()));

  // Track the last animation parameters sent per group so we only send changes.
  private final Map<String, String> lastAnimationKey = new HashMap<>();

  /* Lumyn Device Config creates a config build for everything that
   * LEDs are going to do (ie. zones, channels, animations)\
   */
  private LumynDeviceConfig buildConfig() {
    return new ConfigBuilder()
        .forTeam("401")
        // Channels

        .addChannel(1, "backStrip", 94)
        .addStripZone("leftBack", 47, false)
        .addStripZone("rightBack", 47, true)
        .endChannel()
        .addChannel(2, "frontStrip", 102)
        .addStripZone("leftFront", 51, false)
        .addStripZone("rightFront", 51, true)
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
      Optional<CoordinationLayer> coordinationLayer,
      MatchState matchState) {
    this.driveSubsystem = drive;
    this.hoodSubsystem = hood;
    this.hopperSubsystem = hopper;
    this.shooterSubsystem = shooter;
    this.turretSubsystem = turret;
    this.matchState = matchState;
    this.coordinationLayer = coordinationLayer;

    led.ApplyConfiguration(buildConfig());
    // direct leds
    leftBackLED = led.leds.createDirectLED("leftBack", 47);
    rightBackLED = led.leds.createDirectLED("rightBack", 47);
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
      if (DriverStation.isTeleop()) {
        matchShiftProgressPattern.applyTo(directLEDBackBuffer);
        leftBackLED.update(directLEDBackBuffer);
        rightBackLED.update(directLEDBackBuffer);
      }
      if (DriverStation.isAutonomous()) {

        playMultiFramedAnimationOnce("frontStrip", 100, Animation.Scanner, Color.kYellowGreen);
      }

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
    setAnimationIfDifferent(group, Animation.Fill, Color.kBlack, -1, true);
  }

  public void setGroupColor(String group, Color color) {
    setAnimationIfDifferent(group, Animation.Fill, color, -1, true);
  }

  /**
   * Send an animation to a group only if it's different from the last animation we sent for that
   * group. This prevents resending identical commands every periodic and avoids interrupting
   * non-RunOnce animations on the device.
   *
   * @param group target group name
   * @param animation animation to play
   * @param color optional color (pass Color.kBlack for clear)
   * @param delayMillis delay in ms, -1 for none
   * @param runOnce whether to RunOnce on the device
   */
  // Method created with Copilot
  private void setAnimationIfDifferent(
      String group, Animation animation, Color color, int delayMillis, boolean runOnce) {
    String key =
        animation.name()
            + "|"
            + group
            + "|"
            + (color == null ? "" : color.toString())
            + "|"
            + delayMillis
            + "|"
            + runOnce;
    String last = lastAnimationKey.get(group);
    if (key.equals(last)) {
      return; // same as last sent; skip
    }

    // Build and send the animation chain
    var chain = led.leds.SetAnimation(animation);
    if (color != null) {
      chain = chain.WithColor(color);
    }
    chain = chain.ForGroup(group);
    if (delayMillis >= 0) {
      chain = chain.WithDelay(Milliseconds.of(delayMillis));
    }
    chain.RunOnce(runOnce);

    // update bookkeeping and guard for multi-framed animations
    lastAnimationKey.put(group, key);
    multiFramedAnimationActive = !runOnce;
  }
}
