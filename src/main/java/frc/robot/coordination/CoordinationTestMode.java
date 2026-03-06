package frc.robot.coordination;

import frc.robot.util.TestModeDescription;

public enum CoordinationTestMode implements TestModeDescription {
  ShotTuning("Shot tuning"),
  None; // No test mode selected

  private final String description;

  CoordinationTestMode(String description) {
    this.description = description;
  }

  CoordinationTestMode() {
    this.description = name();
  }

  @Override
  public String getDescription() {
    return this.description;
  }
}
