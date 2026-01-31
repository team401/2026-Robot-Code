package frc.robot.subsystems.intake;

import frc.robot.util.TestModeDescription;

public enum RollerTestMode implements TestModeDescription {
  // Intake Roller-related test modes
  RollerVoltageTuning("Roller Voltage Open Loop Tuning"),
  RollerCurrentTuning("Roller Current Open Loop Tuning"),
  RollerClosedLoopTuning("Roller Closed Loop Tuning"),
  RollerSpeedTuning("Roller Speed Tuning (The target speed the roller should run at)"),
  RollerPhoenixTuning("Roller Phoenix Tuning (no-op)"),
  // Miscellaneous
  None; // No test mode selected

  private final String description;

  RollerTestMode(String description) {
    this.description = description;
  }

  RollerTestMode() {
    this.description = name();
  }

  @Override
  public String getDescription() {
    return this.description;
  }
}
