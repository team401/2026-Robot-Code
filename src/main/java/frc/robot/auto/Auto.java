package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction.AutoActionContext;
import frc.robot.util.ts.GeneratedOptional;

public class Auto {

  private AutoAction rootAction;

  @GeneratedOptional private boolean canBeMirrored = true;
  @GeneratedOptional private boolean shouldBeFlipped = true;

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
