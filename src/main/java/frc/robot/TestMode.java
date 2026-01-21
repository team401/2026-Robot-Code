package frc.robot;

public enum TestMode {
  // Indexer-related test modes
  IndexerVoltageTuning("Indexer Voltage Open Loop Tuning"),
  IndexerCurrentTuning("Indexer Current Open Loop Tuning"),
  IndexerClosedLoopTuning("Indexer Closed Loop Tuning"),
  IndexerPhoenixTuning("Indexer Phoenix Tuning (no-op)"),
  None;

  private final String description;

  TestMode(String description) {
    this.description = description;
  }

  TestMode() {
    this.description = name();
  }

  public String getDescription() {
    return this.description;
  }
}
