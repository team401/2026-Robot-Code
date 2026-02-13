package frc.robot.constants;

import org.littletonrobotics.junction.Logger;

/**
 * FeatureFlags contains the set of flags that enable or disable each subsystem of the robot.
 *
 * <p>When writing a new subsystem, ensure that the program can function with any combination of
 * enabled/disabled subsystems without crashes.
 */
public class FeatureFlags {
  public final Boolean runDrive = false;
  public final Boolean useMAPoseEstimator = false;
  public final Boolean runVision = false;
  public final Boolean runHopper = false;
  public final Boolean runIndexer = false;
  public final Boolean runShooter = false;
  public final Boolean runTurret = false;
  public final Boolean runIntake = false;
  public final Boolean runHood = false;
  public final Boolean useHomingSwitch = false;
  public final Boolean useTuningServer = false;

  /** Print the current state of the feature flags and writes them to the log */
  public void logFlags() {
    System.out.println("Feature flags:");

    System.out.println(" - runDrive: " + runDrive);
    System.out.println(" - useMAPoseEstimator: " + useMAPoseEstimator);
    System.out.println(" - runVision: " + runVision);
    System.out.println(" - runHopper: " + runHopper);
    System.out.println(" - runIndexer: " + runIndexer);
    System.out.println(" - runShooter: " + runShooter);
    System.out.println(" - runTurret: " + runTurret);
    System.out.println(" - runIntake: " + runIntake);
    System.out.println(" - runHood: " + runHood);
    System.out.println(" - useHomingSwitch: " + useHomingSwitch);
    System.out.println(" - useTuningServer: " + useTuningServer);

    Logger.recordOutput("FeatureFlags/runDrive", runDrive);
    Logger.recordOutput("FeatureFlags/useMAPoseEstimator", useMAPoseEstimator);
    Logger.recordOutput("FeatureFlags/runVision", runVision);
    Logger.recordOutput("FeatureFlags/runHopper", runHopper);
    Logger.recordOutput("FeatureFlags/runIndexer", runIndexer);
    Logger.recordOutput("FeatureFlags/runShooter", runShooter);
    Logger.recordOutput("FeatureFlags/runTurret", runTurret);
    Logger.recordOutput("FeatureFlags/runIntake", runIntake);
    Logger.recordOutput("FeatureFlags/runHood", runHood);
    Logger.recordOutput("FeatureFlags/useHomingSwitch", useHomingSwitch);
    Logger.recordOutput("FeatureFlags/useTuningServer", useTuningServer);
  }
}
