package frc.robot.subsystems.hood;

import frc.robot.util.TestModeDescription;

public enum TestMode implements TestModeDescription {
  HoodVoltageTuning("Hood Voltage Open Loop Tuning"),
  HoodCurrentTuning("Hood Current Open Loop Tuning"),
  HoodClosedLoopTuning("Hood Closed Loop Tuning"),
  HoodPhoenixTuning("Hood Phoenix Tuning (no-op)"),
  None;

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
