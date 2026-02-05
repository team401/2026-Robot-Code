package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.sim.ArmSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.DCMotorSimAdapter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.PIDGains;

public class IntakeConstants {
  public final MomentOfInertia pivotInertia = Units.KilogramSquareMeters.of(0.05);
  public final Double pivotGearing = 1.0 / 100.0;
  public final Double armLengthMeters = 0.3;
  public final Double minPivotAngleRadians = -Math.PI / 2;
  public final Double maxPivotAngleRadians = Math.PI / 2;
  public final Double pivotStartingAngleRadians = 0.0;
  public final MomentOfInertia rollersInertia = Units.KilogramSquareMeters.of(0.02);

  // Setpoints for various positions
  public final Angle intakePositionAngle = Radians.of(0.0);
  public final Angle stowPositionAngle = Radians.of(Math.PI / 2);

  // Roller speeds
  public AngularVelocity intakeRollerSpeed = RPM.of(1500.0);

  // Homing parameters
  public final AngularVelocity homingMovementThreshold = RadiansPerSecond.of(0.1);
  public final Time homingTimeoutSeconds = Seconds.of(0.5);
  public final Voltage homingVoltage = Volts.of(2.0);

  // PID GAINS
  public PIDGains pivotPIDGains = new PIDGains(0.5, 0.0, 0.1); // These values are placeholders
  public PIDGains rollersPIDGains = new PIDGains(20.0, 10.0, 10.0); // These values are placeholders

  public MechanismConfig buildPivotMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Intake Pivot Mechanism")
        .withGravityFeedforwardType(GravityFeedforwardType.COSINE_ARM)
        .withLeadMotorId(
            new CANDeviceID(
                JsonConstants.robotInfo.CANBus, JsonConstants.canBusAssignment.intakePivotMotorId))
        .build();
  }

  public TalonFXConfiguration buildPivotTalonFXMotorConfig() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(80.0)))
            .withSlot0(pivotPIDGains.toSlot0Config().withGravityType(GravityTypeValue.Arm_Cosine));
    // Configure motor settings here
    return config;
  }

  public CoppercoreSimAdapter buildPivotSim() {
    return new ArmSimAdapter(
        buildPivotMechanismConfig(),
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            1.0,
            JsonConstants.intakeConstants.pivotInertia.in(Units.KilogramSquareMeters),
            JsonConstants.intakeConstants.armLengthMeters,
            JsonConstants.intakeConstants.minPivotAngleRadians,
            JsonConstants.intakeConstants.maxPivotAngleRadians,
            false,
            JsonConstants.intakeConstants.pivotStartingAngleRadians));
  }

  public TalonFXConfiguration buildRollersTalonFXMotorConfig() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(80.0)))
            .withSlot0(rollersPIDGains.toSlot0Config());
    // Configure motor settings here
    return config;
  }

  public MechanismConfig buildRollersMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Intake Rollers Mechanism")
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(
                JsonConstants.robotInfo.CANBus,
                JsonConstants.canBusAssignment.intakeRollersLeadMotorId))
        .addFollower(
            new CANDeviceID(
                JsonConstants.robotInfo.CANBus,
                JsonConstants.canBusAssignment.intakeRollersFollowerMotorId),
            false)
        .build();
  }

  public CoppercoreSimAdapter buildRollersSim() {
    return new DCMotorSimAdapter(
        buildRollersMechanismConfig(),
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2), rollersInertia.in(KilogramSquareMeters), 1.0),
            DCMotor.getKrakenX60Foc(2),
            0.0,
            0.0));
  }
}
