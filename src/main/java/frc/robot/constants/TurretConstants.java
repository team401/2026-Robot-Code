package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class TurretConstants {

  public final Voltage homingVoltage = Volts.of(-1.0);

  public final Double turretReduction = 37.5;

  public final AngularVelocity homingMovementThreshold = DegreesPerSecond.of(5.0);

  public final Time homingMaxUnmovingTime = Seconds.of(0.5);
  public final Time homingMaxOverallTime = Seconds.of(10.0);

  /**
   * The robot-relative angle that the turret is at once it has homed. This should be determined
   * using CAD to find the angle of the "negative direction" hardstop.
   */
  public final Angle homingAngle = Degrees.zero(); // TODO: Find actual value for this

  public final Integer turretKrakenId = 9; // TODO: Verify this ID

  public final Double turretKP = 0.0;
  public final Double turretKI = 0.0;
  public final Double turretKD = 0.0;
  public final Double turretKS = 0.0;
  public final Double turretKV = 0.0;
  public final Double turretKG = 0.0;
  public final Double turretKA = 0.0;

  public final Double turretExpoKV = 12.0;
  public final Double turretExpoKA = 12.0;

  public MechanismConfig buildMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Turret")
        .withEncoderToMechanismRatio(1.0)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(
            new CANDeviceID(new CANBus(JsonConstants.robotInfo.canivoreBusName), turretKrakenId))
        .build();
  }

  public final Current turretSupplyCurrentLimit = Amps.of(60.0);
  public final Current turretStatorCurrentLimit = Amps.of(40.0);

  public final InvertedValue turretMotorDirection = InvertedValue.CounterClockwise_Positive;

  public TalonFXConfiguration buildTalonFXConfigs() {
    return new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(turretKP)
                .withKI(turretKI)
                .withKD(turretKD)
                .withKS(turretKS)
                .withKV(turretKV)
                .withKG(turretKG)
                .withKA(turretKA))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(turretSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(turretStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withInverted(turretMotorDirection))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicExpo_kA(turretExpoKA)
                .withMotionMagicExpo_kV(turretExpoKV));
  }

  //   public PositionSimAdapter buildTurretSim() {
  //     return new PositionSimAdapter() {

  //     };
  //   }
}
