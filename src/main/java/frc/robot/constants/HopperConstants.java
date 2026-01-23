package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.DiskSim;
import frc.robot.util.DiskSimAdapter;

public class HopperConstants {

  public final Voltage spinningVoltage = Volts.of(0.0); // placeholder

  public final Double hopperReduction = 37.5;

  public final AngularVelocity homingMovementThreshold = DegreesPerSecond.of(2.0);

  public final Time homingMaxUnmovingTime = Seconds.of(5.0);
  public final Time homingMaxOverallTime = Seconds.of(10.0);

  /**
   * The robot-relative angle that the hopper is at once it has homed. This should be determined
   * using CAD to find the angle of the "negative direction" hardstop.
   */
  public final Angle homingAngle = Degrees.zero(); // TODO: Find actual value for this

  public final Integer hopperKrakenId = 11; // TODO: Verify this ID

  public final Double hopperKP = 0.0;
  public final Double hopperKI = 0.0;
  public final Double hopperKD = 0.0;
  public final Double hopperKS = 0.0;
  public final Double hopperKV = 0.0;
  public final Double hopperKG = 0.0;
  public final Double hopperKA = 0.0;

  public final Double hopperExpoKV = 12.0;
  public final Double hopperExpoKA = 12.0;

  public MechanismConfig buildMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Hopper")
        .withEncoderToMechanismRatio(hopperReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(new CANBus(JsonConstants.robotInfo.canivoreBusName), hopperKrakenId))
        .build();
  }

  public final Current hopperSupplyCurrentLimit = Amps.of(60.0);
  public final Current hopperStatorCurrentLimit = Amps.of(40.0);

  public final InvertedValue hopperMotorDirection = InvertedValue.CounterClockwise_Positive;

  public final MomentOfInertia simHopperMOI =
      KilogramSquareMeters.of(
          0.01170557658); // 10 lbs to kg = 4.53592, 2 inches to meters = 0.0508, 4.53592 * 0.0508^2

  // = this number

  public TalonFXConfiguration buildTalonFXConfigs() {
    return new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(hopperKP)
                .withKI(hopperKI)
                .withKD(hopperKD)
                .withKS(hopperKS)
                .withKV(hopperKV)
                .withKG(hopperKG)
                .withKA(hopperKA))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(hopperSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(hopperStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withInverted(hopperMotorDirection))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicExpo_kA(hopperExpoKA)
                .withMotionMagicExpo_kV(hopperExpoKV))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(hopperReduction));
  }

  public DiskSimAdapter buildHopperSim() {
    return new DiskSimAdapter(
        buildMechanismConfig(),
        new DiskSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX60(1), simHopperMOI.in(KilogramSquareMeters), hopperReduction),
            DCMotor.getKrakenX60(1),
            hopperReduction,
            0.0,
            0.0));
  }
}
