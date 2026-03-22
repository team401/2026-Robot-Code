package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import coppercore.parameter_tools.json.annotations.AfterJsonLoad;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFX;
import coppercore.wpilib_interface.subsystems.motors.talonfx.MotorIOTalonFX.SignalRefreshRates;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.PIDGains;
import frc.robot.util.sim.FlywheelSimAdapter;

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

  public PIDGains shooterSlot0Gains = new PIDGains(15.0, 0, 0, 0, 0, 0.55, 0);
  public PIDGains shooterSlot1Gains = new PIDGains(15.0, 0, 0, 0, 0, 0.55, 0);

  /**
   * The number of rotations the shooter motors rotate for each rotation of the output flywheels. If
   * this value is less than one, it is an "upduction" (the flywheels will spin faster than the
   * motors).
   */
  public final Double shooterReduction = 5.0 / 8.0;

  /** Per-motor supply current limit */
  public final Current shooterSupplyCurrentLimit = Amps.of(40.0);

  /** Per-motor stator current limit */
  public final Current shooterStatorCurrentLimit = Amps.of(80.0);

  public final InvertedValue shooterMotorDirection =
      InvertedValue.CounterClockwise_Positive; // TODO: Actual value
  // TODO: Actual invert values for the follower
  public final Boolean invertFollower = true;

  public final SignalRefreshRates shooterSignalRefreshRates =
      new SignalRefreshRates(Hertz.of(200.0), Hertz.of(20.0), Hertz.of(1000.0));

  /**
   * The rate at which to run the velocity and follower control requests for the shooter motors.
   * Note that if this doesn't match the "output" rate in shooterSignalRefreshRates, this number
   * won't fully take effect.
   */
  public final Frequency shooterClosedLoopFrequency = Hertz.of(1000);

  @JSONExclude
  public final int shooterMediumPrioritySignals =
      MotorIOTalonFX.DEFAULT_MEDIUM_PRIORITY_SIGNALS
          | MotorIOTalonFX.DEFAULT_HIGH_PRIORITY_SIGNALS & ~MotorIOTalonFX.SIGNAL_VELOCITY;

  @JSONExclude public final int shooterHighPrioritySignals = MotorIOTalonFX.SIGNAL_VELOCITY;
  @JSONExclude public final int shooterOutputSignals = MotorIOTalonFX.DEFAULT_OUTPUT_SIGNALS;

  // Perhaps regenerative braking can improve our battery performance?
  public final NeutralModeValue defaultShooterNeutralMode = NeutralModeValue.Brake;

  public AngularVelocity shooterMaxVelocity = RPM.of(2900); // TODO: Real value
  public AngularAcceleration shooterMaxAcceleration = RPM.of(3000).per(Second);

  @JSONExclude public Velocity<VoltageUnit> characterizationRampRate = Volts.of(0.1).per(Second);

  public final Time velocityFilterTime = Seconds.of(0.01);

  /**
   * When the shooter's velocity is within shooterVelocitySetpointEpsilon of its target velocity, it
   * is considered "at its setpoint"
   */
  public AngularVelocity shooterVelocitySetpointEpsilon = RPM.of(50);

  /**
   * When the shooter's velocity is within the shooterPassingVelocitySetpointEpsilon of its target
   * velocity, it is close enough to pass
   */
  public AngularVelocity shooterPassingVelocitySetpointEpsilon = RPM.of(100.0);

  /**
   * The debounce time for the falling edge debouncer that smooths the "is at setpoint velocity"
   * signal; this value allows us to continue shooting despite dropping shooter velocity (trusting
   * our recovery time) but a value that is too long will result in shooting some shots while the
   * flywheels are genuinely at the wrong speed.
   */
  public Time atSetpointDebounceTime = Seconds.of(0.2);

  /**
   * When the shooter less than shooterSlot0Epsilon less than its closed loop reference, it will use
   * slot 0. When it's more than shooterSlot0Epsilon below its setpoint, it will use slot 1.
   *
   * <p>This was tested and does help recovery time (~5 fuel per second -> ~12 fuel per second
   * recovery rate)
   */
  public AngularVelocity shooterSlot0Epsilon = RPM.of(150);

  /** The time it takes for closed loop to ramp from 0 to 300 amps of requested output */
  public Time torqueCurrentRampTime = Seconds.of(1);

  public TalonFXConfiguration buildTalonFXConfigs() {
    return new TalonFXConfiguration()
        .withSlot0(shooterSlot0Gains.toSlot0Config())
        .withSlot1(shooterSlot1Gains.toSlot1Config())
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
                .withSensorToMechanismRatio(shooterReduction)
                .withVelocityFilterTimeConstant(velocityFilterTime))
        .withTorqueCurrent(new TorqueCurrentConfigs().withPeakReverseTorqueCurrent(Amps.zero()))
        .withClosedLoopRamps(
            new ClosedLoopRampsConfigs().withTorqueClosedLoopRampPeriod(torqueCurrentRampTime));
  }

  public MechanismConfig buildMechanismConfig() {
    CANBus bus = JsonConstants.robotInfo.CANBus;
    return MechanismConfig.builder()
        .withName("Shooter")
        .withEncoderToMechanismRatio(shooterReduction)
        .withMotorToEncoderRatio(1.0)
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .withLeadMotorId(new CANDeviceID(bus, JsonConstants.canBusAssignment.shooterLeaderId))
        .addFollower(
            new CANDeviceID(bus, JsonConstants.canBusAssignment.shooterFollowerId), invertFollower)
        .build();
  }

  // TODO: Find a reasonable value for this MOI
  public final MomentOfInertia shooterMOI = KilogramSquareMeters.of(0.0049163462);

  public CoppercoreSimAdapter buildShooterSim() {
    return new FlywheelSimAdapter(
        buildMechanismConfig(),
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(3), shooterMOI.in(KilogramSquareMeters), 1.0),
            DCMotor.getKrakenX60Foc(3)));
  }
}
