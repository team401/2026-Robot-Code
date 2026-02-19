package frc.robot.subsystems.intake;

import frc.robot.util.TestModeDescription;

public enum RollerTestMode implements TestModeDescription {
  // Intake Roller-related test modes
  RollerVoltageTuning("Voltage"),
  RollerCurrentTuning("Current"),
  RollerClosedLoopTuning("Closed Loop"),
  RollerSpeedTuning("Speed Tuning"),
  RollerPhoenixTuning("Phoenix"),
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
