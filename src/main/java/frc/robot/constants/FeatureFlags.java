package frc.robot.constants;

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
}
