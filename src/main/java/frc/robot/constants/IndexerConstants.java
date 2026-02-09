package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.DCMotorSimAdapter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.PIDGains;

public class IndexerConstants {

  public final Double indexerReduction = 1.0; // TODO: find actual value

  // Constant for maximum relative velocity error between the desired motor output and the actual
  // motor output
  public final Double indexerMaximumRelativeVelocityError = 0.01;

  public final Boolean indexerDemoMode = false;

  public PIDGains indexerGains = new PIDGains(10.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public MotionProfileConfig indexerMotionProfileConfig =
      MotionProfileConfig.immutable(
          RotationsPerSecond.zero(),
          RotationsPerSecondPerSecond.of(0.0),
          RotationsPerSecondPerSecond.of(0.0).div(Seconds.of(1.0)),
          Volts.zero().div(RotationsPerSecond.of(1)),
          Volts.zero().div(RotationsPerSecondPerSecond.of(1)));

  public Double indexerMaxAccelerationRotationsPerSecondSquared =
      3000.0; // TODO: Find actual values
  public Double indexerMaxJerkRotationsPerSecondCubed = 1000.0;

  public MechanismConfig buildMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Indexer")
        .withEncoderToMechanismRatio(indexerReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(
                JsonConstants.robotInfo.CANBus, JsonConstants.canBusAssignment.indexerKrakenId))
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
        .withSlot0(indexerGains.toSlot0Config())
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(indexerSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(indexerStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withInverted(indexerMotorDirection))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(indexerMaxAccelerationRotationsPerSecondSquared)
                .withMotionMagicJerk(indexerMaxJerkRotationsPerSecondCubed));
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
