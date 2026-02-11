package frc.robot.subsystems.intake;

import frc.robot.util.TestModeDescription;

public enum PivotTestMode implements TestModeDescription {
  // Intake Pivot-related test modes
  PivotVoltageTuning("Pivot Voltage Open Loop Tuning"),
  PivotCurrentTuning("Pivot Current Open Loop Tuning"),
  PivotClosedLoopTuning("Pivot Closed Loop Tuning"),
  PivotPhoenixTuning("Pivot Phoenix Tuning (no-op)"),
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
