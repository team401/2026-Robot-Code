package frc.robot.subsystems.intake;

import frc.robot.util.TestModeDescription;

public enum PivotTestMode implements TestModeDescription {
  // Intake Pivot-related test modes
  PivotVoltageTuning("Voltage"),
  PivotCurrentTuning("Current"),
  PivotClosedLoopTuning("Closed Loop"),
  PivotPhoenixTuning("Phoenix"),
  // Miscellaneous
  None; // No test mode selected

  private final String description;

  PivotTestMode(String description) {
    this.description = description;
  }

  PivotTestMode() {
    this.description = name();
  }

  @Override
  public String getDescription() {
    return this.description;
  }
}
