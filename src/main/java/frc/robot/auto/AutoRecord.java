package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Holds information about the auto
 *
 * @name name of the auto
 * @isBlue alliance of the auto, can be null
 * @isLeft mirroring of the auto, can be null
 * @command the command of the auto
 */
public record AutoRecord(String name, Boolean isBlue, Boolean isLeft, Command command) {
  /**
   * Holds information about the auto
   *
   * @name name of the auto
   * @command the command of the auto
   *     <p>Alliance is set to null. Mirroring is set to null.
   */
  public AutoRecord(String name, Command command) {
    this(name, null, null, command);
  }
}
