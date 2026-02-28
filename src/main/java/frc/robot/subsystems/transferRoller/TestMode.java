package frc.robot.subsystems.transferRoller;

import frc.robot.util.TestModeDescription;

public enum TestMode implements TestModeDescription {
  // Intake Roller test modes
  TransferRollerVoltageTuning("TransferRoller Voltage Open Loop Tuning"),
  TransferRollerCurrentTuning("TransferRoller Current Open Loop Tuning"),
  TransferRollerClosedLoopTuning("TransferRoller Closed Loop Tuning"),
  TransferRollerPhoenixTuning("TransferRoller Phoenix Tuning (no-op)"),
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
