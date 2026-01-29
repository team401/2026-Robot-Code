package frc.robot.subsystems.intake;

import frc.robot.util.TestModeDescription;

public enum TestMode implements TestModeDescription {
  // Intake-related test modes
  // TODO: add intake-specific test modes
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
