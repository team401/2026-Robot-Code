package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction.AutoActionContext;

public class Auto {

  private AutoAction rootAction;

  private boolean canBeMirrored = true;
  private boolean shouldBeFlipped = true;

  public Command toCommand(AutoActionContext data) {
    return rootAction.toCommand(data);
  }

  public boolean canMirror() {
    return canBeMirrored;
  }

  public boolean shouldFlip() {
    return shouldBeFlipped;
  }
}
