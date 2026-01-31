package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import coppercore.wpilib_interface.subsystems.sim.ArmSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.CoppercoreSimAdapter;
import coppercore.wpilib_interface.subsystems.sim.DCMotorSimAdapter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeConstants {
    public final MomentOfInertia pivotInertia = Units.KilogramSquareMeters.of(0.05);
    public final Double pivotGearing = 1.0 / 100.0;
    public final Double armLengthMeters = 0.3;
    public final Double minPivotAngleRadians = -Math.PI / 2;
    public final Double maxPivotAngleRadians = Math.PI / 2;
    public final Double pivotStartingAngleRadians = 0.0;
    public final MomentOfInertia rollersInertia = Units.KilogramSquareMeters.of(0.02);

    // Setpoints for various positions
    public final Angle intakePositionAngle = Radians.of(0.0);
    public final Angle stowPositionAngle = Radians.of(Math.PI / 2);

    // Roller speeds
    public final Double intakeRollerSpeedRPM = 1500.0;

    // Homing parameters
    public final AngularVelocity homingMovementThreshold= RadiansPerSecond.of(0.1);
    public final Time homingTimeoutSeconds = Seconds.of(0.5);
    public final Voltage homingVoltage = Volts.of(2.0);

    // CAN IDs
    public final Integer intakePivotMotorId = 30;
    public final Integer intakeRollersLeadMotorId = 31;
    public final Integer intakeRollersFollowerMotorId = 32;

    public MechanismConfig buildPivotMechanismConfig() {
        return MechanismConfig.builder()
            .withName("Intake Pivot Mechanism")
            .withGravityFeedforwardType(GravityFeedforwardType.COSINE_ARM)
            .withLeadMotorId(
                new CANDeviceID(
                    new CANBus(JsonConstants.robotInfo.canivoreBusName),
                    intakePivotMotorId))
            .build();
    }

    public TalonFXConfiguration buildPivotTalonFXMotorConfig() {
        TalonFXConfiguration config =
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(Amps.of(80.0)))
                .withSlot0(
                    new Slot0Configs()
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKS(0.0)
                        .withKV(0.0)
                        .withKA(0.0)
                        .withKG(0.0)
                        .withKP(0.5)
                        .withKI(0.0)
                        .withKD(0.1));
        // Configure motor settings here
        return config;
    }

    public CoppercoreSimAdapter buildPivotSim() {
        return new ArmSimAdapter(
            buildPivotMechanismConfig(),
            new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                1.0,
                JsonConstants.intakeConstants.pivotInertia.in(Units.KilogramSquareMeters),
                JsonConstants.intakeConstants.armLengthMeters,
                JsonConstants.intakeConstants.minPivotAngleRadians,
                JsonConstants.intakeConstants.maxPivotAngleRadians,
                false,
                JsonConstants.intakeConstants.pivotStartingAngleRadians));
    }

    public TalonFXConfiguration buildRollersTalonFXMotorConfig() {
        TalonFXConfiguration config =
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(Amps.of(80.0)))
                .withSlot0(
                    new Slot0Configs()
                        .withKS(0.0)
                        .withKV(0.0)
                        .withKA(0.0)
                        .withKG(10.0)
                        .withKP(20.0)
                        .withKI(10.0)
                        .withKD(10.0));
        // Configure motor settings here
        return config;
    }

    public MechanismConfig buildRollersMechanismConfig() {
        return MechanismConfig.builder()
            .withName("Intake Rollers Mechanism")
            .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
            .withLeadMotorId(
                new CANDeviceID(
                    new CANBus(JsonConstants.robotInfo.canivoreBusName),
                    intakeRollersLeadMotorId))
            .addFollower(new CANDeviceID(
                    new CANBus(JsonConstants.robotInfo.canivoreBusName),
                    intakeRollersFollowerMotorId),
                    false)
            .build();
    }

    public CoppercoreSimAdapter buildRollersSim() {
        return new DCMotorSimAdapter(
            buildRollersMechanismConfig(),
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2),
                    rollersInertia.in(KilogramSquareMeters),
                    1.0),
                DCMotor.getKrakenX60Foc(2),
                0.0,
                0.0));
    }
}
