package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;

public record AutoRecord(String name, Boolean isBlue, Boolean isLeft, Command command) {

  public AutoRecord(String name, Command command) {
    this(name, null, null, command);
  }
}
