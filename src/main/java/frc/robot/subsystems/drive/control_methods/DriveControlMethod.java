package frc.robot.subsystems.drive.control_methods;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public abstract class DriveControlMethod {

  public final String name;
  protected final Drive drive;
  private final boolean requiresEnabled;

  public String getName() {
    return name;
  }

  public DriveControlMethod(Drive drive, String name) {
    this(drive, name, true);
  }

  public DriveControlMethod(Drive drive, String name, boolean requiresEnabled) {
    this.drive = drive;
    this.requiresEnabled = requiresEnabled;
    this.name = name;
  }

  /**
   * This method can be used to log warnings related to this control method, for example if a
   * invalid path is given to the linear drive control method, it can log a warning that the path is
   * invalid and the control method is not working
   *
   * @param warning_message
   */
  public void warn(String warning_message) {
    Logger.recordOutput("driveControlMethod/warning", warning_message);
  }

  /**
   * This method is called when the control method is enabled. This can be used to initialize any
   * state or variables related to the control method.
   */
  public void activateControl() {}

  /**
   * This method is called when the control method is disabled. This can be used to clean up any
   * state or variables related to the control method.
   */
  public void deactivateControl() {}

  public final void periodic() {
    if (requiresEnabled && !DriverStation.isEnabled()) {
      return;
    }
    _periodic();
  }

  protected abstract void _periodic();
}
