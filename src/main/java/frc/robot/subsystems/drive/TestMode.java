package frc.robot.subsystems.drive;

import frc.robot.util.TestModeDescription;

public enum TestMode implements TestModeDescription {
  // Drive-related test modes
  DriveGainsTuning("Drive Steering and Drive Gains Tuning"),
  // Miscellaneous
  None; // No test mode selected

  private final String description;

  TestMode(String description) {
    this.description = description;
  }

  TestMode() {
    this.description = name();
  }

  @Override
  public String getDescription() {
    return this.description;
  }
}
