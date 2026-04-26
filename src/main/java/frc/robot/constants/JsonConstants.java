package frc.robot.constants;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.therekrab.autopilot.APTarget;
import coppercore.parameter_tools.json.JSONHandler;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.json.adapters.measure.JSONMeasure;
import coppercore.parameter_tools.json.helpers.JSONConverter;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import coppercore.wpilib_interface.controllers.Controllers;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.Auto;
import frc.robot.auto.AutoAction;
import frc.robot.auto.Autos;
import frc.robot.constants.drive.DriveConstants;
import frc.robot.constants.drive.PhysicalDriveConstants;
import frc.robot.util.json.JSONAPTarget;
import frc.robot.util.json.JSONMotionProfileConfig;
import frc.robot.util.json.JSONRotation2d;
import frc.robot.util.json.JSONRotation3d;
import frc.robot.util.json.JSONTransform2d;
import frc.robot.util.json.JSONTransform3d;
import frc.robot.util.json.OptionalTypeAdapterFactory;
import frc.robot.util.ts.PythonGenerator;
import frc.robot.util.ts.PythonGeometryMethods;

/**
 * JsonConstants handles loading and saving of all constants through JSON. Call `loadConstants`
 * before using constants.
 */
public class JsonConstants {
  public static EnvironmentHandler environmentHandler;
  public static JSONHandler jsonHandler;

  // TODO: Figure out a better way to serialize the MotionProfileConfig
  static {
    JSONMeasure.registerUnit(RotationsPerSecondPerSecond.per(Second));
    // This should be replaced with a polymorphic adapter in the future
    // But that requires changes to the coppercore library
    JSONConverter.addConversion(MotionProfileConfig.class, JSONMotionProfileConfig.class);

    JSONConverter.addConversion(Transform2d.class, JSONTransform2d.class);
    JSONConverter.addConversion(Transform3d.class, JSONTransform3d.class);
    JSONConverter.addConversion(Rotation2d.class, JSONRotation2d.class);
    JSONConverter.addConversion(Rotation3d.class, JSONRotation3d.class);
    JSONConverter.addConversion(APTarget.class, JSONAPTarget.class);

    JSONMeasure.registerUnit(Amp.per(Second));
    JSONMeasure.registerUnit(RPM.per(Second), "RPM Per Second");
  }

  public static JSONHandler loadConstants(RobotContainer robotContainer) {

    environmentHandler =
        EnvironmentHandler.getEnvironmentHandler(
            Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());

    var jsonSyncSettings = new JSONSyncConfigBuilder();

    Controllers.applyControllerConfigToBuilder(jsonSyncSettings);

    jsonSyncSettings.addJsonTypeAdapterFactory(new OptionalTypeAdapterFactory());

    jsonSyncSettings.setUpPolymorphAdapter(AutoAction.class);

    var pathProvider = environmentHandler.getEnvironmentPathProvider();

    System.out.println("[JsonConstants] Environment name: " + pathProvider.getEnvironmentName());
    jsonHandler = new JSONHandler(jsonSyncSettings.build(), pathProvider);

    robotInfo = jsonHandler.getObject(new RobotInfo(), "RobotInfo.json");
    aprilTagConstants = jsonHandler.getObject(new AprilTagConstants(), "AprilTagConstants.json");
    featureFlags = jsonHandler.getObject(new FeatureFlags(), "FeatureFlags.json");
    canBusAssignment = jsonHandler.getObject(new CANBusAssignment(), "CANBusAssignment.json");
    driveConstants = jsonHandler.getObject(new DriveConstants(), "DriveConstants.json");

    // jsonHandler.saveObject(new VisionConstants(), "VisionConstants.json");
    visionConstants = jsonHandler.getObject(new VisionConstants(), "VisionConstants.json");

    physicalDriveConstants =
        jsonHandler.getObject(new PhysicalDriveConstants(), "PhysicalDriveConstants.json");
    operatorConstants = jsonHandler.getObject(new OperatorConstants(), "OperatorConstants.json");
    hopperConstants = jsonHandler.getObject(new HopperConstants(), "HopperConstants.json");
    indexerConstants = jsonHandler.getObject(new IndexerConstants(), "IndexerConstants.json");
    turretConstants = jsonHandler.getObject(new TurretConstants(), "TurretConstants.json");
    intakeConstants = jsonHandler.getObject(new IntakeConstants(), "IntakeConstants.json");
    shooterConstants = jsonHandler.getObject(new ShooterConstants(), "ShooterConstants.json");
    hoodConstants = jsonHandler.getObject(new HoodConstants(), "HoodConstants.json");
    climberConstants = jsonHandler.getObject(new ClimberConstants(), "ClimberConstants.json");
    transferRollerConstants =
        jsonHandler.getObject(new TransferRollerConstants(), "TransferRollerConstants.json");
    shotMaps = jsonHandler.getObject(new ShotMaps(), "ShotMaps.json");

    redFieldLocations =
        jsonHandler.getObject(new FieldLocationInstance(), "RedFieldLocations.json");
    blueFieldLocations =
        jsonHandler.getObject(new FieldLocationInstance(), "BlueFieldLocations.json");
    manualModeConstants =
        jsonHandler.getObject(new ManualModeConstants(), "ManualModeConstants.json");
    strategyConstants = jsonHandler.getObject(new StrategyConstants(), "StrategyConstants.json");

    autos = jsonHandler.getObject(new Autos(), "Autos.json");

    if (featureFlags.useTuningServer) {
      // do not crash Robot if routes could not be added for any reason
      try {
        jsonHandler.addRoute("/hopper", hopperConstants);
        jsonHandler.addRoute("/indexer", indexerConstants);
        jsonHandler.addRoute("/turret", turretConstants);
        jsonHandler.addRoute("/shooter", shooterConstants);
        jsonHandler.addRoute("/drive", driveConstants);
        jsonHandler.addRoute("/hood", hoodConstants);
        jsonHandler.addRoute("/transferroller", transferRollerConstants);
        jsonHandler.addRoute("/intake", intakeConstants);
        jsonHandler.addRoute("/climber", climberConstants);
        jsonHandler.addRoute("/vision", visionConstants);
        jsonHandler.registerPostCallback(
            "/vision",
            (visionConstants) -> {
              System.out.println("Vision Constants were updated");
              return true;
            });
        jsonHandler.addRoute("/autos", autos);
        jsonHandler.registerPostCallback(
            "/autos",
            (autos) -> {
              robotContainer.loadAutoCommands();
              return true;
            });
        jsonHandler.addRoute("/shotmaps", shotMaps);
        jsonHandler.addRoute("/manualMode", manualModeConstants);
      } catch (Exception ex) {
        System.err.println("could not add routes for constant tuning: " + ex);
      }
    }

    controllers =
        jsonHandler.getObject(new Controllers(), operatorConstants.controllerBindingsFile);

    if (Constants.currentMode == Constants.Mode.SIM) {
      PythonGeometryMethods.registerAll();
      PythonGenerator.generateForClasses(
          "auto_action.py",
          Auto.class,
          Transform2d.class,
          Transform3d.class,
          Rotation2d.class,
          Rotation3d.class,
          Pose2d.class,
          Pose3d.class,
          Translation2d.class,
          Translation3d.class);
    }
    return jsonHandler;
  }

  public static RobotInfo robotInfo;
  public static AprilTagConstants aprilTagConstants;
  public static CANBusAssignment canBusAssignment;
  public static FeatureFlags featureFlags;
  public static DriveConstants driveConstants;
  public static VisionConstants visionConstants;
  public static PhysicalDriveConstants physicalDriveConstants;
  public static OperatorConstants operatorConstants;
  public static HopperConstants hopperConstants;
  public static IndexerConstants indexerConstants;
  public static TurretConstants turretConstants;
  public static IntakeConstants intakeConstants;
  public static ShooterConstants shooterConstants;
  public static HoodConstants hoodConstants;
  public static TransferRollerConstants transferRollerConstants;
  public static ShotMaps shotMaps;
  public static FieldLocationInstance redFieldLocations;
  public static FieldLocationInstance blueFieldLocations;
  public static ClimberConstants climberConstants;
  public static ManualModeConstants manualModeConstants;
  public static StrategyConstants strategyConstants;

  public static Autos autos;

  public static Controllers controllers;
}
