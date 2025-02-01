package frc.robot.subsytems.swerve;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsytems.swerve.SwerveModule.SwerveEncoder;

public class SparkMaxAbsVoltageEncoder implements SwerveEncoder {
    private SparkMax spark;
    private double voltageOffset;

    public SparkMaxAbsVoltageEncoder(SparkMax spark, double voltageOffset) {
        this.spark = spark;
        this.voltageOffset = voltageOffset;
    }

    public static double scale(double value, double min, double max, double a, double b) {
        return a + ((value - min) / (max - min)) * (b - a);
    }

    @Override
    public Rotation2d getAbsAngle() {
        double voltage = spark.getAnalog().getVoltage();
        return Rotation2d.fromRotations(scale(voltage - voltageOffset, 0, 5, 0, 1));
    }
}