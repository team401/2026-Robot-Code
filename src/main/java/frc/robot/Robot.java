// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import coppercore.wpilib_interface.subsystems.StatusSignalRefresher;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.JsonConstants;
import frc.robot.util.TotalCurrentCalculator;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Disable CTRE hoot logging to prevent overruns
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    if (!FeatureFlags.usePhoenixDiagnosticServer) {
      // Setting a negative value here disables/stops the server
      Unmanaged.setPhoenixDiagnosticsStartTime(-1);
      // This got our mean cycle time (admittedly while not seeing any tags) down to 15ms
    }

    DriverStation.silenceJoystickConnectionWarning(true);

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    if (Constants.currentMode == Constants.Mode.SIM && JsonConstants.robotInfo.runAutoTesting) {
      autoTestingSimulation = new AutoTestingSimulation(this);
    }
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Refresh all status signals, must be done before any IOs run updateInputs
    long refresherStartTimeUs = RobotController.getFPGATime();
    StatusSignalRefresher.refreshAll();
    long refresherEndTimeUs = RobotController.getFPGATime();
    if (JsonConstants.featureFlags.logPeriodicTiming) {
      Logger.recordOutput(
          "PeriodicTime/statusSignalRefresherMs",
          (refresherEndTimeUs - refresherStartTimeUs) / 1000.0);
    }

    // Poll for and process any outstanding HTTP requests
    // This action's performance is logged inside only if the featureflag is enabled
    robotContainer.processHTTPRequests();

    // Log current draw in replay mode
    TotalCurrentCalculator.periodic();

    // Run the DependencyOrderedExecutor, which executes all registered actions in order so that all
    // dependencies are satisfied.
    // One of these actions runs the CommandScheduler. This is responsible for polling buttons,
    // adding newly-scheduled commands, running already-scheduled commands, removing finished or
    // interrupted commands, and running subsystem periodic() methods.  This must be called from the
    // robot's periodic block in order for anything in the Command-based framework to work.
    // The CommandScheduler action is added here: src/main/java/frc/robot/RobotContainer.java:58
    robotContainer.getDependencyOrderedExecutor().execute();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    if (Constants.currentMode == Constants.Mode.SIM && JsonConstants.robotInfo.runAutoTesting) {
      autoTestingSimulation.update();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // This ensures that the higher, default drive current limit used in auto is reduced for teleop.
    robotContainer.lowerDriveSupplyCurrentLimit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.updateRobotModel();
  }

  private AutoTestingSimulation autoTestingSimulation = null;

  private static class AutoTestingSimulation {

    public static final String AUTO_TESTING_PREFIX = "AutoTesting/";

    // This class is responsible for running the auto testing simulation. It will listen to network
    // tables for commands to start and stop the simulation, and for what alliance station to start
    // from. It will then run the appropriate simulation based on the selected command and alliance
    // station.

    LoggedDashboardChooser<AllianceStationID> allianceStationChooser;
    Robot robot;

    public AutoTestingSimulation(Robot robot) {
      this.robot = robot;
      // Initialize the auto testing simulation, set up network table listeners, etc.
      allianceStationChooser =
          new LoggedDashboardChooser<>(AUTO_TESTING_PREFIX + "AllianceStation");
      allianceStationChooser.addDefaultOption("Unknown", AllianceStationID.Unknown);
      allianceStationChooser.addOption("Red 1", AllianceStationID.Red1);
      allianceStationChooser.addOption("Red 2", AllianceStationID.Red2);
      allianceStationChooser.addOption("Red 3", AllianceStationID.Red3);
      allianceStationChooser.addOption("Blue 1", AllianceStationID.Blue1);
      allianceStationChooser.addOption("Blue 2", AllianceStationID.Blue2);
      allianceStationChooser.addOption("Blue 3", AllianceStationID.Blue3);

      SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "RobotEnabled", false);
      SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "AutoEnabled", false);

      SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "StartAuto", false);
      SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "StopAuto", false);

      SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "SetPos", false);
      SmartDashboard.putNumber(AUTO_TESTING_PREFIX + "SetPosX", 0.0);
      SmartDashboard.putNumber(AUTO_TESTING_PREFIX + "SetPosY", 0.0);
      SmartDashboard.putNumber(AUTO_TESTING_PREFIX + "SetPosTheta", 0.0);

      DriverStationSim.setDsAttached(true);
    }

    public void update() {

      DriverStationSim.setAllianceStationId(allianceStationChooser.get());
      DriverStationSim.notifyNewData();

      if (SmartDashboard.getBoolean(AUTO_TESTING_PREFIX + "StartAuto", false)) {
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "StartAuto", false);
      }

      if (SmartDashboard.getBoolean(AUTO_TESTING_PREFIX + "StopAuto", false)) {
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(false);
        SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "StopAuto", false);
      }

      if (SmartDashboard.getBoolean(AUTO_TESTING_PREFIX + "SetPos", false)) {
        double x = SmartDashboard.getNumber(AUTO_TESTING_PREFIX + "SetPosX", 0.0);
        double y = SmartDashboard.getNumber(AUTO_TESTING_PREFIX + "SetPosY", 0.0);
        double theta = SmartDashboard.getNumber(AUTO_TESTING_PREFIX + "SetPosTheta", 0.0);
        robot
            .robotContainer
            .getDriveSubsystem()
            .setPose(new Pose2d(x, y, Rotation2d.fromDegrees(theta)));
        SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "SetPos", false);
      }

      DriverStationSim.notifyNewData();

      SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "RobotEnabled", DriverStation.isEnabled());
      SmartDashboard.putBoolean(AUTO_TESTING_PREFIX + "AutoEnabled", DriverStation.isAutonomous());
    }
  }
}
