package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.PIDGains;

public class HopperConstants {

  public final Voltage dejamVoltage = Volts.of(3.0); // placeholder

  public final Boolean hopperDemoMode = true;

  public final Double hopperReduction = 1.0;

  public final AngularVelocity spinningMovementThreshold = RadiansPerSecond.of(3.0);

  public PIDGains hopperGains = PIDGains.kPID(16.0, 0.0, 0.0);

  // The important values here are maxAcceleration and maxJerk because it uses
  // a profiled velocity request
  public MotionProfileConfig hopperMotionProfileConfig =
      MotionProfileConfig.immutable(
          RotationsPerSecond.zero(),
          RotationsPerSecondPerSecond.of(3000.0),
          RotationsPerSecondPerSecond.of(1000.0).div(Seconds.of(1.0)),
          Volts.zero().div(RotationsPerSecond.of(1)),
          Volts.zero().div(RotationsPerSecondPerSecond.of(1)));

  public MechanismConfig buildMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Hopper")
        .withEncoderToMechanismRatio(hopperReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(
                JsonConstants.robotInfo.CANBus, JsonConstants.canBusAssignment.hopperKrakenId))
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
        .withSlot0(hopperGains.toSlot0Config())
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(hopperSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(hopperStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withInverted(hopperMotorDirection))
        .withMotionMagic(hopperMotionProfileConfig.asMotionMagicConfigs());
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
