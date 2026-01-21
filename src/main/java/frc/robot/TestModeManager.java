package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class TestModeManager {
  // This class should not be instantiated
  private TestModeManager() {}

  private static LoggedDashboardChooser<TestMode> testModeChooser = null;

  /**
   * Initialize the Test Mode Chooser.
   *
   * <p>Should be called by the RobotContainer at some point during initialization
   */
  public static void init() {
    if (testModeChooser != null) {
      Exception e = new Exception("TestModeManager.init() called more than once!");
      e.printStackTrace();
      return;
    }

    testModeChooser = new LoggedDashboardChooser<>("Test Mode Selector");

    for (TestMode mode : TestMode.values()) {
      if (mode == TestMode.None) {
        testModeChooser.addDefaultOption(mode.getDescription(), mode);
      } else {
        testModeChooser.addOption(mode.getDescription(), mode);
      }
    }
  }

  /**
   * Return the current Test Mode from the Test Mode Chooser.
   *
   * <p>If the TestModeManager has not been initialized or the robot is not in Test mode, this will
   * return TestMode.None
   */
  public static TestMode getTestMode() {
    if (testModeChooser == null || !DriverStation.isTest()) {
      return TestMode.None;
    }

    return testModeChooser.get();
  }
}
