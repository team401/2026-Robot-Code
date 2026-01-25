package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * A test mode manager adds a LoggedDashboardChooser for a set of test modes described by an enum.
 * One of the enum values must be `None`. The enum class must implement TestModeDescription.
 */
public final class TestModeManager<TestMode extends Enum<?> & TestModeDescription> {
  private String prefix;
  private TestMode None;
  private LoggedDashboardChooser<TestMode> testModeChooser = null;

  /**
   * Construct a new test mode manager.
   *
   * <p>Should be called by each subsystem at some point during initialization
   *
   * @param prefix - the prefix used to display in dashboard chooser
   * @param enumClazz - the enum class containing the test modes
   */
  public TestModeManager(String prefix, Class<TestMode> enumClazz) {
    this.prefix = prefix;
    this.testModeChooser = new LoggedDashboardChooser<>(prefix + " Test Mode Selector");
    for (TestMode mode : enumClazz.getEnumConstants()) {
      if ("None".equals(mode.name())) {
        None = mode;
        testModeChooser.addDefaultOption(mode.getDescription(), mode);
      } else {
        testModeChooser.addOption(mode.getDescription(), mode);
      }
    }
    if (None == null)
      throw new RuntimeException("Test mode class " + enumClazz + " must have a None variant");
  }

  /**
   * Return the current Test Mode from the Test Mode Chooser.
   *
   * <p>If the TestModeManager has not been initialized or the robot is not in Test mode, this will
   * return TestMode.None
   */
  public TestMode getTestMode() {
    if (!DriverStation.isTest()) {
      return this.None;
    }

    return testModeChooser.get();
  }
}
