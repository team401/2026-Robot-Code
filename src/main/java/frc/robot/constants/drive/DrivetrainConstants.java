package frc.robot.constants.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class DrivetrainConstants {

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
        Volts.of(0.2)
    );


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

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType =
        DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType =
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


    public void finishLoadingConstants() {
        // Nothing to do here yet
    }

    private record DrivetrainMotorConfig(
        ClosedLoopOutputType closedLoopOutputType,
        Double gearRatio,
        MomentOfInertia momentOfInertia,
        Voltage frictionVoltage 
    ) {

    }
}
