package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.DCMotorSimAdapter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerConstants {

  public final Double indexerReduction = 1.0; // TODO: find actual value

  // Constant for maximum relative velocity error between the desired motor output and the actual
  // motor output
  public final Double indexerMaximumRelativeVelocityError = 0.01;

  public final Integer indexerKrakenId = 10; // TODO: Verify this ID

  public final Double indexerKP = 0.0;
  public final Double indexerKI = 0.0;
  public final Double indexerKD = 0.0;
  public final Double indexerKS = 0.0;
  public final Double indexerKV = 0.0;
  public final Double indexerKG = 0.0;
  public final Double indexerKA = 0.0;

  public final Double indexerExpoKV = 12.0;
  public final Double indexerExpoKA = 12.0;

  public MechanismConfig buildMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Indexer")
        .withEncoderToMechanismRatio(indexerReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(
            GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(new CANBus(JsonConstants.robotInfo.canivoreBusName), indexerKrakenId))
        .build();
  }

  // TODO: Find actual values
  public final Current indexerSupplyCurrentLimit = Amps.of(60.0);
  public final Current indexerStatorCurrentLimit = Amps.of(40.0);
  public final MomentOfInertia simIndexerMOI = KilogramSquareMeters.of(0.00025);

  public final InvertedValue indexerMotorDirection =
      InvertedValue.CounterClockwise_Positive; // TODO: find value

  public TalonFXConfiguration buildTalonFXConfigs() {
    return new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(indexerKP)
                .withKI(indexerKI)
                .withKD(indexerKD)
                .withKS(indexerKS)
                .withKV(indexerKV)
                .withKG(indexerKG)
                .withKA(indexerKA))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(indexerSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(indexerStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withInverted(indexerMotorDirection))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicExpo_kA(indexerExpoKA)
                .withMotionMagicExpo_kV(indexerExpoKV));
  }

  public CoppercoreSimAdapter buildIndexerSim() {
    return new DCMotorSimAdapter(
        buildMechanismConfig(),
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44Foc(1),
                simIndexerMOI.in(KilogramSquareMeters),
                1 / indexerReduction),
            DCMotor.getKrakenX44Foc(1),
            0.0,
            0.0));
  }
}
