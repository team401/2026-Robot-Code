package frc.robot.constants.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;

public class DrivetrainConstants {

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;


    public final Distance wheelRadius = Inches.of(2);

    public final DrivetrainMotorConfig driveMotorConfig = new DrivetrainMotorConfig(
        ClosedLoopOutputType.Voltage,
        6.026785714285714,
        KilogramSquareMeters.of(0.01),
        Volts.of(0.2)
    );

    public final DrivetrainMotorConfig steerMotorConfig = new DrivetrainMotorConfig(
        ClosedLoopOutputType.Voltage,
        26.09090909090909,
        KilogramSquareMeters.of(0.01),
        Volts.of(0.2),
        StaticFeedforwardSignValue.UseClosedLoopSign
    );

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The type of motor used for the drive motor
    private final DriveMotorArrangement kDriveMotorType =
        DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private final SteerMotorArrangement kSteerMotorType =
        SteerMotorArrangement.TalonFX_Integrated;

    public final ModuleConfig frontLeftModule = new ModuleConfig()
        .withDriveMotorId(7)
        .withSteerMotorId(2)
        .withEncoderId(9)
        .withEncoderOffset(Rotations.of(0.113525390625))
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withXPos(Inches.of(10.875))
        .withYPos(Inches.of(10.875));

    public final ModuleConfig frontRightModule = new ModuleConfig()
        .withDriveMotorId(1)
        .withSteerMotorId(6)
        .withEncoderId(12)
        .withEncoderOffset(Rotations.of(-0.136474609375))
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withXPos(Inches.of(10.875))
        .withYPos(Inches.of(-10.875));

    public final ModuleConfig backLeftModule = new ModuleConfig()
        .withDriveMotorId(5)
        .withSteerMotorId(8)
        .withEncoderId(10)
        .withEncoderOffset(Rotations.of(0.0986328125))
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withXPos(Inches.of(-10.875))
        .withYPos(Inches.of(10.875));

    public final ModuleConfig backRightModule = new ModuleConfig()
        .withDriveMotorId(3)
        .withSteerMotorId(4)
        .withEncoderId(11)
        .withEncoderOffset(Rotations.of(-0.180908203125))
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withXPos(Inches.of(-10.875))
        .withYPos(Inches.of(-10.875));
    
    private final Current kSlipCurrent = Amps.of(120);

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.12);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private final Double kCoupleRatio = 3.857142857142857;

    private final Boolean kInvertLeftSide = true;
    private final Boolean kInvertRightSide = false;

    private final Integer kPigeonId = 13;

    private record DrivetrainMotorConfig(
        ClosedLoopOutputType closedLoopOutputType,
        Double gearRatio,
        MomentOfInertia momentOfInertia,
        Voltage frictionVoltage,
        StaticFeedforwardSignValue staticFeedforwardSignValue
    ) {
        public DrivetrainMotorConfig(
            ClosedLoopOutputType closedLoopOutputType,
            Double gearRatio,
            MomentOfInertia momentOfInertia,
            Voltage frictionVoltage
        ) {
            this(
                closedLoopOutputType,
                gearRatio,
                momentOfInertia,
                frictionVoltage,
                StaticFeedforwardSignValue.UseVelocitySign
            );
        }
    }

    public Slot0Configs driveGains;
    public Slot0Configs steerGains;

    public SwerveDrivetrainConstants DrivetrainConstants;

    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft;
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight;
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft;
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight;
    

    public void finishLoadingConstants() {
        driveGains = JsonConstants.driveConstants.driveGains.toSlot0Config()
            .withStaticFeedforwardSign(driveMotorConfig.staticFeedforwardSignValue());

        steerGains = JsonConstants.driveConstants.steerGains.toSlot0Config()
            .withStaticFeedforwardSign(steerMotorConfig.staticFeedforwardSignValue());

        DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName(JsonConstants.robotInfo.kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);
        var constantCreator =
                new SwerveModuleConstantsFactory<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(driveMotorConfig.gearRatio())
                    .withSteerMotorGearRatio(steerMotorConfig.gearRatio())
                    .withCouplingGearRatio(kCoupleRatio)
                    .withWheelRadius(wheelRadius)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorClosedLoopOutput(steerMotorConfig.closedLoopOutputType())
                    .withDriveMotorClosedLoopOutput(driveMotorConfig.closedLoopOutputType())
                    .withSlipCurrent(kSlipCurrent)
                    .withSpeedAt12Volts(kSpeedAt12Volts)
                    .withDriveMotorType(kDriveMotorType)
                    .withSteerMotorType(kSteerMotorType)
                    .withFeedbackSource(kSteerFeedbackType)
                    .withDriveMotorInitialConfigs(driveInitialConfigs)
                    .withSteerMotorInitialConfigs(steerInitialConfigs)
                    .withEncoderInitialConfigs(encoderInitialConfigs)
                    .withSteerInertia(steerMotorConfig.momentOfInertia())
                    .withDriveInertia(driveMotorConfig.momentOfInertia())
                    .withSteerFrictionVoltage(steerMotorConfig.frictionVoltage())
                    .withDriveFrictionVoltage(driveMotorConfig.frictionVoltage());
        FrontLeft = frontLeftModule.toSwerveModuleConstants(constantCreator, kInvertLeftSide);
        FrontRight = frontRightModule.toSwerveModuleConstants(constantCreator, kInvertRightSide);
        BackLeft = backLeftModule.toSwerveModuleConstants(constantCreator, kInvertLeftSide);
        BackRight = backRightModule.toSwerveModuleConstants(constantCreator, kInvertRightSide);
    }
}
