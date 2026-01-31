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
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.HardstoppedDCMotorSimAdapter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretConstants {

  public final Voltage homingVoltage = Volts.of(-3.0);

  public final Double turretReduction = 37.5;

  public final AngularVelocity homingMovementThreshold = DegreesPerSecond.of(2.0);

  public final Time homingMaxUnmovingTime = Seconds.of(5.0);
  public final Time homingMaxOverallTime = Seconds.of(10.0);

  /**
   * The robot-relative angle that the turret is at once it has homed. This should be determined
   * using CAD to find the angle of the "negative direction" hardstop.
   */
  public final Angle homingAngle = Degrees.zero(); // TODO: Find actual value for this

  public final Integer turretKrakenId = 9; // TODO: Verify this ID

  // TODO: Root cause why turret sim requires such ridiculous gains to function properly
  // These gains are CRAZY. MAKE SURE that you change these gains before deploying to a robot, or it
  // will definitely break.
  // The current gains are by no means perfect, but they do make the turret track a goal position
  // decently in sim. Once we get a physical turret mechanism, the sim can be modified to closely
  // follow the behavior of the real life mechanism and then all will be more accurate.
  public Double turretKP = 5000.0; // 40
  public Double turretKI = 0.0;
  public Double turretKD = 800.0; // 600
  public Double turretKS = 0.0;
  public Double turretKV = 0.0;
  public Double turretKG = 0.0;
  public Double turretKA = 80.0; // 55

  public Double turretExpoKV = 2.0;
  public Double turretExpoKA = 37.5;

  public MechanismConfig buildMechanismConfig() {
    return MechanismConfig.builder()
        .withName("Turret")
        .withEncoderToMechanismRatio(turretReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(new CANDeviceID(JsonConstants.robotInfo.CANBus, turretKrakenId))
        .build();
  }

  public final Current turretSupplyCurrentLimit = Amps.of(60.0);
  public final Current turretStatorCurrentLimit = Amps.of(80.0);

  public final InvertedValue turretMotorDirection = InvertedValue.CounterClockwise_Positive;

  // According to Shorya, 422 alum/mentor:
  // "we usually just figure out what we expect out of the subsystems
  // and just tune the moi as a magic constant"
  public final MomentOfInertia simTurretMOI = KilogramSquareMeters.of(0.00025);
  // 10 lbs to kg = 4.53592, 2 inches to meters = 0.0508, 4.53592 *
  // 0.0508^2. These calculations seemed to create a VERY heavy object

  public final Angle minTurretAngle = Degrees.of(-5.0);
  public final Angle maxTurretAngle = Degrees.of(350);

  // = this number

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
                .withMotionMagicExpo_kV(turretExpoKV))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(turretReduction));
  }

  public CoppercoreSimAdapter buildTurretSim() {
    return new HardstoppedDCMotorSimAdapter(
        buildMechanismConfig(),
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44Foc(1),
                simTurretMOI.in(KilogramSquareMeters),
                1 / turretReduction),
            DCMotor.getKrakenX44Foc(1),
            0.0,
            0.0),
        minTurretAngle,
        maxTurretAngle);
  }
}
