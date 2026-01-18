package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import edu.wpi.first.units.measure.Current;

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
        // .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(
                new CANBus("canivore"),
                indexerKrakenId)) // TODO: replace with robotInfo.canivoreBusName
        .build();
  }

  public final Current indexerSupplyCurrentLimit = Amps.of(60.0); // TODO: Find actual values
  public final Current indexerStatorCurrentLimit = Amps.of(40.0);

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
}
