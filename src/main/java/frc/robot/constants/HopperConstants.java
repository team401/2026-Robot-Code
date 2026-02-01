package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperConstants {

  public final Voltage dejamVoltage = Volts.of(3.0); // placeholder

  public final Boolean hopperDemoMode = true;

  public final Double hopperReduction = 1.0;

  public final AngularVelocity spinningMovementThreshold = RadiansPerSecond.of(3.0);

  public final Time spinningMaxUnmovingTime = Seconds.of(5.0);
  public final Time spinningMaxOverallTime = Seconds.of(10.0);

  /**
   * The robot-relative angle that the hopper is at once it has homed. This should be determined
   * using CAD to find the angle of the "negative direction" hardstop.
   */
  public Double hopperKP = 16.0;

  public Double hopperKI = 0.0;
  public Double hopperKD = 0.0;
  public Double hopperKS = 0.0;
  public Double hopperKV = 0.0;
  public Double hopperKG = 0.0;
  public Double hopperKA = 0.0;

  public Double hopperMaxAccelerationRotationsPerSecondSquared = 3000.0; // TODO: Find actual values
  public Double hopperMaxJerkRotationsPerSecondCubed = 1000.0;

  public MechanismConfig buildMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Hopper")
        .withEncoderToMechanismRatio(hopperReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(
                new CANBus(JsonConstants.robotInfo.canivoreBusName),
                JsonConstants.canBusAssignment.hopperKrakenId))
        .build();
  }

  public final Current hopperSupplyCurrentLimit = Amps.of(60.0);
  public final Current hopperStatorCurrentLimit = Amps.of(40.0);
  public final Current dejamCurrentThreshold =
      hopperStatorCurrentLimit.minus(Amps.of(5.0)); // TODO: Figure out how the current limits work

  public final InvertedValue hopperMotorDirection = InvertedValue.CounterClockwise_Positive;

  public final MomentOfInertia simHopperMOI = KilogramSquareMeters.of(0.00025);

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
                .withMotionMagicAcceleration(hopperMaxAccelerationRotationsPerSecondSquared)
                .withMotionMagicJerk(hopperMaxJerkRotationsPerSecondCubed));
  }

  public CoppercoreSimAdapter buildHopperSim() {
    return new DCMotorSimAdapter(
        buildMechanismConfig(),
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                simHopperMOI.in(KilogramSquareMeters),
                1 / hopperReduction),
            DCMotor.getKrakenX60(1),
            hopperReduction,
            0.0));
  }
}
