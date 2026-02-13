package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.ElevatorMechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.ElevatorSimAdapter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

// Copilot was used to help write this file
public class ClimberConstants {

  public final Voltage homingVoltage = Volts.of(-3.0);

  public final Double climberReduction = 37.5;

  public final AngularVelocity homingMovementThreshold = DegreesPerSecond.of(2.0);

  public final Time homingMaxUnmovingTime = Seconds.of(5.0);

  public final Angle homingAngle = Degrees.zero(); // TODO: Find actual value for this
  public final Angle upperClimbAngle =
      homingAngle.plus(Degrees.of(90.0)); // TODO: Find actual value for this

  public Double climberKP = 0.0; // Tune these
  public Double climberKI = 0.0;
  public Double climberKD = 0.0;
  public Double climberKS = 0.0;
  public Double climberKV = 0.0;
  public Double climberKG = 0.0;
  public Double climberKA = 0.0;

  public Double climberExpoKV = 2.0;
  public Double climberExpoKA = 37.5;

  public ElevatorMechanismConfig buildMechanismConfig() {
    return ElevatorMechanismConfig.builder()
        .withName("climber")
        .withEncoderToMechanismRatio(climberReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(
                JsonConstants.robotInfo.CANBus, JsonConstants.canBusAssignment.climberKrakenId))
        .build();
  }

  public final Current climberSupplyCurrentLimit = Amps.of(60.0);
  public final Current climberStatorCurrentLimit = Amps.of(80.0);

  public final InvertedValue climberMotorDirection = InvertedValue.CounterClockwise_Positive;

  // According to Shorya, 422 alum/mentor:
  // "we usually just figure out what we expect out of the subsystems
  // and just tune the moi as a magic constant"
  public final MomentOfInertia simClimberMOI = KilogramSquareMeters.of(0.00025);
  // 10 lbs to kg = 4.53592, 2 inches to meters = 0.0508, 4.53592 *
  // 0.0508^2. These calculations seemed to create a VERY heavy object

  public final double minClimberHeightMeters = 0.0;
  public final double maxClimberHeightMeters = 0.0; // TODO: Find actual value
  public final double climberStartingHeightMeters = 0.0;
  public final double climberMeasurementStdDevs = 0.001;

  public TalonFXConfiguration buildTalonFXConfigs() {
    return new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(climberKP)
                .withKI(climberKI)
                .withKD(climberKD)
                .withKS(climberKS)
                .withKV(climberKV)
                .withKG(climberKG)
                .withKA(climberKA))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(climberSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(climberStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withInverted(climberMotorDirection))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicExpo_kA(climberExpoKA)
                .withMotionMagicExpo_kV(climberExpoKV))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(climberReduction));
  }

  public CoppercoreSimAdapter buildClimberSim() {
    return new ElevatorSimAdapter(
        buildMechanismConfig(),
        new ElevatorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                simClimberMOI.in(KilogramSquareMeters),
                1 / climberReduction),
            DCMotor.getKrakenX60Foc(1),
            minClimberHeightMeters,
            maxClimberHeightMeters,
            true,
            climberStartingHeightMeters,
            climberMeasurementStdDevs));
  }
}
