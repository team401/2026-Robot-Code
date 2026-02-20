package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.PIDGains;

public class IntakeConstants {

  // Gearing constants
  public final Double pivotReduction = 42.5;
  public final Double rollersReduction = 2.0;

  // Pivot mechanism constants
  // These values are placeholders and should be updated with real values
  public final Distance armLength = Meters.of(0.3);
  public final Angle minPivotAngle = Degrees.of(0.0);
  public final Angle maxPivotAngle = Degrees.of(90.0);
  public final Angle pivotStartingAngle = Degrees.of(90.0);

  // Sim Constants
  public final MomentOfInertia rollersInertia = Units.KilogramSquareMeters.of(0.02);
  public final MomentOfInertia pivotInertia = Units.KilogramSquareMeters.of(0.05);

  // Setpoint for various positions
  public final Angle intakePositionAngle = Degrees.of(0.0);
  public final Angle stowPositionAngle = Degrees.of(90.0);

  // Roller speeds
  public AngularVelocity intakeRollerSpeed = RPM.of(1500.0);

  // Homing parameters
  public final AngularVelocity homingMovementThreshold = RadiansPerSecond.of(0.1);
  public final Time homingTimeoutSeconds = Seconds.of(0.5);
  public final Voltage homingVoltage = Volts.of(-2.0);

  // PID GAINS
  public PIDGains pivotPIDGains = PIDGains.kPID(0.5, 0.0, 0.1); // These values are placeholders
  public PIDGains rollersPIDGains =
      PIDGains.kPID(20.0, 10.0, 10.0); // These values are placeholders

  // Current Limits (These current limits are placeholders and were picked randomly)
  public final Current pivotSupplyCurrentLimit = Amps.of(40.0);
  public final Current pivotStatorCurrentLimit = Amps.of(40.0);
  public final Current rollersStatorCurrentLimit = Amps.of(40.0);
  public final Current rollersSupplyCurrentLimit = Amps.of(40.0);

  public MechanismConfig buildPivotMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Intake/Pivot")
        .withGravityFeedforwardType(GravityFeedforwardType.COSINE_ARM)
        .withEncoderToMechanismRatio(pivotReduction)
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
                    .withSupplyCurrentLimit(pivotSupplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(pivotStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(
                pivotPIDGains
                    .toSlot0Config()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(pivotReduction));
    // Configure motor settings here
    return config;
  }

  public CoppercoreSimAdapter buildPivotSim() {
    return new ArmSimAdapter(
        buildPivotMechanismConfig(),
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            pivotReduction,
            pivotInertia.in(Units.KilogramSquareMeters),
            armLength.in(Meters),
            minPivotAngle.in(Radians),
            maxPivotAngle.in(Radians),
            true,
            pivotStartingAngle.in(Radians)));
  }

  public TalonFXConfiguration buildRollersTalonFXMotorConfig() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(rollersSupplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(rollersStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(rollersPIDGains.toSlot0Config())
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(rollersReduction));
    // Configure motor settings here
    return config;
  }

  public MechanismConfig buildRollersMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Intake/Rollers")
        .withEncoderToMechanismRatio(rollersReduction)
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
    DCMotor motor = DCMotor.getKrakenX60Foc(2);
    return new DCMotorSimAdapter(
        buildRollersMechanismConfig(),
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, rollersInertia.in(KilogramSquareMeters), 1.0),
            motor));
  }
}
