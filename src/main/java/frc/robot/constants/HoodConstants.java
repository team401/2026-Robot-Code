package frc.robot.constants;

public class HoodConstants {
  /**
   * How long the hood motor IO must report a disconnected state before displaying the alert. This
   * should be long enough that momentary disconnects (e.g. from going over the bump) don't trigger
   * the alert, but actual disconnects don't go unnoticed for a long time.
   */
  public final Double disconnectedDebounceTimeSeconds = 1.0;
}
