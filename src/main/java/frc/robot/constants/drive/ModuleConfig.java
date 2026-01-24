package frc.robot.constants.drive;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ModuleConfig {
    public Integer driveMotorId;
    public Integer steerMotorId;
    public Integer encoderId;
    public Angle encoderOffset;
    public Boolean steerMotorInverted;
    public Boolean encoderInverted;
    public Distance xPos;
    public Distance yPos;

    public ModuleConfig withDriveMotorId(Integer id) {
        this.driveMotorId = id;
        return this;
    }

    public ModuleConfig withSteerMotorId(Integer id) {
        this.steerMotorId = id;
        return this;
    }

    public ModuleConfig withEncoderId(Integer id) {
        this.encoderId = id;
        return this;
    }

    public ModuleConfig withEncoderOffset(Angle offset) {
        this.encoderOffset = offset;
        return this;
    }

    public ModuleConfig withSteerMotorInverted(Boolean inverted) {
        this.steerMotorInverted = inverted;
        return this;
    }

    public ModuleConfig withEncoderInverted(Boolean inverted) {
        this.encoderInverted = inverted;
        return this;
    }

    public ModuleConfig withXPos(Distance x) {
        this.xPos = x;
        return this;
    }

    public ModuleConfig withYPos(Distance y) {
        this.yPos = y;
        return this;
    }
    
}