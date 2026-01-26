package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import coppercore.parameter_tools.json.annotations.AfterJsonLoad;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.DCMotorSimAdapter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterConstants {
  public final Double[] distanceToViDistancesMeters = {1.8, 2.0, 3.5};
  public final Double[] distanceToViVisMetersPerSecond = {6.4, 6.8, 7.5};

  /**
   * The "default initial velocity" that is used if getViFromDistance is called without first
   * initializing the map.
   */
  public final Double defaultViMetersPerSecond = 9.0;

  @JSONExclude private InterpolatingDoubleTreeMap distanceToVi;

  /** Initialize the map of distance to initial velocity */
  @AfterJsonLoad
  public void initializeViMap() {
    if (distanceToViDistancesMeters.length != distanceToViVisMetersPerSecond.length) {
      throw new IllegalArgumentException(
          "distanceToViDistancesMeters and distanceToViVisMetersPerSecond had different lengths in ShooterConstants");
    }

    distanceToVi = new InterpolatingDoubleTreeMap();
    for (int i = 0; i < distanceToViDistancesMeters.length; i++) {
      distanceToVi.put(distanceToViDistancesMeters[i], distanceToViVisMetersPerSecond[i]);
    }
  }

  public double getViFromDistance(double distanceMeters) {
    if (distanceToVi == null) {
      System.err.println(
          "Error: ShooterConsants.getViFromDistance called before map was initialized.");
      return defaultViMetersPerSecond;
    }

    return distanceToVi.get(distanceMeters);
  }

  public final Integer shooterLeadMotorId = 14;
  public final Integer shooterTopFollowerMotorId = 15;
  public final Integer shooterBottomFollowerMotorId = 16;

  public final Double shooterKP = 0.0;
  public final Double shooterKI = 0.0;
  public final Double shooterKD = 0.0;
  public final Double shooterKS = 0.0;
  public final Double shooterKV = 0.0;
  public final Double shooterKA = 0.0;

  /**
   * The number of rotations the shooter motors rotate for each rotation of the output flywheels. If
   * this value is less than one, it is an "upduction" (the flywheels will spin faster than the
   * motors).
   */
  public final Double shooterReduction = 5.0 / 8.0;

  /** Per-motor supply current limit */
  public final Current shooterSupplyCurrentLimit = Amps.of(40.0);

  /** Per-motor stator current limit */
  public final Current shooterStatorCurrentLimit = Amps.of(40.0);

  public final InvertedValue shooterMotorDirection =
      InvertedValue.CounterClockwise_Positive; // TODO: Actual value
  // TODO: Actual invert values for the followers
  public final Boolean invertTopFollower = true;
  public final Boolean invertBottomFollower = true;

  // Perhaps regenerative braking can improve our battery performance?
  public final NeutralModeValue defaultShooterNeutralMode = NeutralModeValue.Brake;

  public final AngularVelocity shooterMaxVelocity = RPM.of(2900); // TODO: Real value
  public final AngularAcceleration shooterMaxAcceleration = RPM.of(1000).per(Second);

  public TalonFXConfiguration buildTalonFXConfigs() {
    return new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(shooterKP)
                .withKI(shooterKI)
                .withKD(shooterKD)
                .withKS(shooterKS)
                .withKG(0.0)
                .withKV(shooterKV)
                .withKA(shooterKA))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(shooterStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(shooterSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(shooterMotorDirection)
                .withNeutralMode(defaultShooterNeutralMode))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(shooterMaxAcceleration)
                .withMotionMagicCruiseVelocity(shooterMaxVelocity))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(shooterReduction));
  }

  public MechanismConfig buildMechanismConfig() {
    CANBus bus = new CANBus(JsonConstants.robotInfo.canivoreBusName);
    return MechanismConfig.builder()
        .withName("Shooter")
        .withEncoderToMechanismRatio(shooterReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(new CANDeviceID(bus, shooterLeadMotorId))
        .addFollower(new CANDeviceID(bus, shooterTopFollowerMotorId), invertTopFollower)
        .addFollower(new CANDeviceID(bus, shooterBottomFollowerMotorId), invertBottomFollower)
        .build();
  }

  // TODO: Find a reasonable value for this MOI
  public final MomentOfInertia shooterMOI = KilogramSquareMeters.of(0.01);

  public CoppercoreSimAdapter buildShooterSim() {
    return new DCMotorSimAdapter(
        buildMechanismConfig(),
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(3),
                shooterMOI.in(KilogramSquareMeters),
                1 / shooterReduction),
            DCMotor.getKrakenX60Foc(3)));
  }
}
