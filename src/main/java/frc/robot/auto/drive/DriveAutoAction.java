package frc.robot.auto.drive;

import frc.robot.auto.AutoAction;
import frc.robot.util.ts.GeneratedDefault;

public abstract class DriveAutoAction extends AutoAction {

  // Just tells us if we can mirror for things
  // Because for things like climb we can not mirror the climb poses
  @GeneratedDefault("True")
  public boolean canMirror = true;
}
