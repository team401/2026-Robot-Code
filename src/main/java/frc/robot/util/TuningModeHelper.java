package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.function.Consumer;


import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableMotionProfile.MotionProfileConsumer;
import frc.robot.util.LoggedTunablePIDGains.GainsConsumer;

public class TuningModeHelper<TestModeEnum extends Enum<TestModeEnum> & TestModeDescription> {

    private EnumMap<TestModeEnum, TuningMode> tuningModes;

    public TuningModeHelper(Class<TestModeEnum> testModeEnumClass) {
        tuningModes = new EnumMap<>(testModeEnumClass);
    }

    public TuningModeHelper<TestModeEnum> addTuningMode(TestModeEnum testMode, TuningMode tuningMode) {
        tuningModes.put(testMode, tuningMode);
        return this;
    }

    public void runTestMode(TestModeEnum testMode) {
        var tuningMode = tuningModes.get(testMode);
        if (tuningMode != null) {
            tuningMode.tuningPeriodic();
        }
    }

    public TuningModeHelper<TestModeEnum> addStandardTuningModesForMotor(TestModeEnum phoenixTuningMode, TestModeEnum voltageTuningMode, TestModeEnum currentTuningMode, String prefix, MotorIO motorIO) {
        addTuningMode(phoenixTuningMode, EMPTY);
        addTuningMode(voltageTuningMode, addOpenLoopVoltageTuning(builder(), prefix, motorIO, v -> {}).build());
        addTuningMode(currentTuningMode, addOpenLoopCurrentTuning(builder(), prefix, motorIO, c -> {}).build());
        return this;
    }

    public static class TuningMode {

        private List<TunableValueUpdate> loggedTunableValues;

        protected TuningMode(List<TunableValueUpdate> loggedTunableValues) {
            this.loggedTunableValues = loggedTunableValues;
        }

        public void tuningPeriodic() {
            for (var tunableValue : loggedTunableValues) {
                tunableValue.runIfChanged(hashCode());
            }
        }

    }

    public static class TuningModeBuilder {
        private List<TunableValueUpdate> loggedTunableValues;

        public TuningModeBuilder() {
            this.loggedTunableValues = new ArrayList<>();
        }

        public TuningModeBuilder addTunableValueUpdates(TunableValueUpdate... tunableValueUpdate) {
            for (var tunableValue : tunableValueUpdate) {
                this.loggedTunableValues.add(tunableValue);
            }
            return this;
        }

        public TuningModeBuilder addLoggedTunableNumbers(Consumer<double[]> consumer, LoggedTunableNumber... tunableNumbers) {
            this.loggedTunableValues.add(id -> LoggedTunableNumber.ifChanged(id, consumer, tunableNumbers));
            return this;
        }

        public TuningModeBuilder addLoggedTunablePIDGains(LoggedTunablePIDGains tunableGains, LoggedTunablePIDGains.GainsConsumer consumer) {
            this.loggedTunableValues.add(id -> tunableGains.ifChanged(id, consumer));
            return this;
        }

        public TuningModeBuilder addLoggedTunableMotionProfile(LoggedTunableMotionProfile tunableMotionProfile, LoggedTunableMotionProfile.MotionProfileConsumer consumer) {
            this.loggedTunableValues.add(id -> tunableMotionProfile.ifChanged(id, consumer));
            return this;
        }

        public <M extends MutableMeasure<U, B, M>, B extends Measure<U>, U extends Unit> TuningModeBuilder 
                addLoggedTunableMeasure(LoggedTunableMeasure<M, B, U> tunableMeasure, LoggedTunableMeasure.MeasureConsumer<M, B, U> consumer) {
            this.loggedTunableValues.add(id -> tunableMeasure.ifChanged(id, consumer));
            return this;
        }

        public TuningMode build() {
            return new TuningMode(loggedTunableValues);
        }
    }

    // TODO: Figure out what and how to name the prefixs and locations for the logged tunables

    public static TuningModeBuilder builder() {
        return new TuningModeBuilder();
    }

    public static TuningModeBuilder addOpenLoopVoltageTuning(TuningModeBuilder builder, String prefix, MotorIO motorIO, Consumer<Voltage> onChangeCallback) {
        return builder
            .addLoggedTunableMeasure(
                LoggedTunableMeasure.VOLTAGE.of(
                    prefix + "OpenLoopVoltageTuningVolts",
                    Volts.mutable(0.0)
                ),
                voltage -> {
                    onChangeCallback.accept(voltage);
                    motorIO.controlOpenLoopVoltage(voltage);
                }
            );
    }

    public static TuningModeBuilder addOpenLoopCurrentTuning(TuningModeBuilder builder, String prefix, MotorIO motorIO, Consumer<Current> onChangeCallback) {
        return builder
            .addLoggedTunableMeasure(
                LoggedTunableMeasure.CURRENT.of(
                    prefix + "OpenLoopCurrentTuningAmps",
                    Amps.mutable(0.0)
                ),
                current -> {
                    onChangeCallback.accept(current);
                    motorIO.controlOpenLoopCurrent(current);
                }
            );
    }


    public static TuningModeBuilder addClosedLoopPIDGainsTuning(
        TuningModeBuilder builder,
        String prefix,
        PIDGains defaultGains,
        GainsConsumer onChangeCallback,
        MotorIO... motorIOs
    ) {
        var tunableGains = new LoggedTunablePIDGains(prefix + "ClosedLoopPIDGains", defaultGains);
        return builder
            .addLoggedTunablePIDGains(
                tunableGains,
                onChangeCallback
                    .chain(tunableGains.getMotorIOAppliers(motorIOs))
            );
    }

    public static TuningModeBuilder addClosedLoopProfileTuning(TuningModeBuilder builder, String prefix, MotionProfileConfig defaultConfig, MotionProfileConsumer onChangeCallback, MotorIO... motorIOs) {
        LoggedTunableMotionProfile tunableMotionProfile = new LoggedTunableMotionProfile(prefix + "ClosedLoopMotionProfile", defaultConfig);
        return builder.addLoggedTunableMotionProfile(
                tunableMotionProfile,
                onChangeCallback
                    .chain(tunableMotionProfile.getMotorIOAppliers(motorIOs))
            );
    }

    public static TuningModeBuilder addClosedLoopPositionUnprofiledTuning(
        TuningModeBuilder builder,
        String prefix,
        PIDGains defaultGains,
        Angle defaultTargetPosition,
        GainsConsumer gainsOnChangeCallback,
        Consumer<Angle> targetPositionOnChangeCallback,
        MotorIO... motorIOs
    ) {
        return addClosedLoopPIDGainsTuning(builder, prefix, defaultGains, gainsOnChangeCallback, motorIOs)
            .addLoggedTunableMeasure(
                LoggedTunableMeasure.ANGLE.of(
                    prefix + "ClosedLoopPositionUnprofiledTargetAngleRadians",
                    defaultTargetPosition,
                    Radians
                ),
                targetAngle -> {
                    targetPositionOnChangeCallback.accept(targetAngle);
                    for (var motorIO : motorIOs) {
                        motorIO.controlToPositionUnprofiled(targetAngle);
                    }
                }
            );
    }

    public static TuningModeBuilder addClosedLoopVelocityUnprofiledTuning(
        TuningModeBuilder builder,
        String prefix,
        PIDGains defaultGains,
        AngularVelocity defaultTargetVelocity,
        GainsConsumer gainsOnChangeCallback,
        Consumer<AngularVelocity> targetVelocityOnChangeCallback,
        MotorIO... motorIOs
    ) {
        return addClosedLoopPIDGainsTuning(builder, prefix, defaultGains, gainsOnChangeCallback, motorIOs)
            .addLoggedTunableMeasure(
                LoggedTunableMeasure.ANGULAR_VELOCITY.of(
                    prefix + "ClosedLoopVelocityUnprofiledTargetVelocityRPM",
                    defaultTargetVelocity,
                    RPM
                ),
                targetVelocity -> {
                    targetVelocityOnChangeCallback.accept(targetVelocity);
                    for (var motorIO : motorIOs) {
                        motorIO.controlToVelocityUnprofiled(targetVelocity);
                    }
                }
            );
    }

    public static TuningModeBuilder addClosedLoopPositionProfiledTuning(
        TuningModeBuilder builder,
        String prefix,
        PIDGains defaultGains,
        MotionProfileConfig defaultMotionProfileConfig,
        Angle defaultTargetPosition,
        GainsConsumer gainsOnChangeCallback,
        MotionProfileConsumer profileOnChangeCallback,
        MotorIO... motorIOs
    ) {

        addClosedLoopPIDGainsTuning(builder, prefix, defaultGains, gainsOnChangeCallback, motorIOs);
        addClosedLoopProfileTuning(builder, prefix, defaultMotionProfileConfig, profileOnChangeCallback, motorIOs);
        return builder
            .addLoggedTunableMeasure(
                LoggedTunableMeasure.ANGLE.of(
                    prefix + "ClosedLoopPositionProfiledTargetAngleRadians",
                    defaultTargetPosition,
                    Radians
                ),
                targetAngle -> {
                    for (var motorIO : motorIOs) {
                        motorIO.controlToPositionProfiled(targetAngle);
                    }
                }
            );
    }

    public static TuningModeBuilder addClosedLoopVelocityProfiledTuning(
        TuningModeBuilder builder,
        String prefix,
        PIDGains defaultGains,
        MotionProfileConfig defaultMotionProfileConfig,
        AngularVelocity defaultTargetVelocity,
        GainsConsumer gainsOnChangeCallback,
        MotionProfileConsumer profileOnChangeCallback,
        MotorIO... motorIOs
    ) {
        addClosedLoopPIDGainsTuning(builder, prefix, defaultGains, gainsOnChangeCallback, motorIOs);
        addClosedLoopProfileTuning(builder, prefix, defaultMotionProfileConfig, profileOnChangeCallback, motorIOs);
        return builder
            .addLoggedTunableMeasure(
                LoggedTunableMeasure.ANGULAR_VELOCITY.of(
                    prefix + "ClosedLoopVelocityProfiledTargetVelocityRPM",
                    defaultTargetVelocity,
                    RPM
                ),
                targetVelocity -> {
                    for (var motorIO : motorIOs) {
                        motorIO.controlToVelocityProfiled(targetVelocity);
                    }
                }
            );
    }

    public static final TuningMode EMPTY = new TuningModeBuilder().build();

    public static final TuningMode phoenixTuning(){
        return EMPTY;
    }

    @FunctionalInterface
    public static interface TunableValueUpdate {
        void runIfChanged(int id);
    }
}
