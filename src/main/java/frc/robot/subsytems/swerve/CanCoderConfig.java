package frc.robot.subsytems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsytems.swerve.SwerveModule.SwerveEncoder;

public class CanCoderConfig implements SwerveEncoder {
    private final CANcoder absoluteEncoder;

    public CanCoderConfig(int encoderId, double magnetOffset) {
        absoluteEncoder = new CANcoder(encoderId);

        var config = new MagnetSensorConfigs();
        config.withAbsoluteSensorDiscontinuityPoint(0.5);
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        if (magnetOffset <= 0) {
            config.MagnetOffset = (-magnetOffset) - .5;
        } else {
            config.MagnetOffset = (-magnetOffset) + .5;
        }

        absoluteEncoder.getConfigurator().apply(config);
        absoluteEncoder.getAbsolutePosition().setUpdateFrequency(100, 250);
    }

    @Override
    public Rotation2d getAbsAngle() {
        return Rotation2d.fromRadians(
                absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2);
    }
}