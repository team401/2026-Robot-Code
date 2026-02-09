package frc.robot.subsystems.indexer;

import frc.robot.util.TestModeDescription;

public enum TestMode implements TestModeDescription {
  // Indexer-related test modes
  IndexerVoltageTuning("Indexer Voltage Open Loop Tuning"),
  IndexerCurrentTuning("Indexer Current Open Loop Tuning"),
  IndexerClosedLoopTuning("Indexer Closed Loop Tuning"),
  IndexerPhoenixTuning("Indexer Phoenix Tuning (no-op)"),
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
