package frc.robot.subsytems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
        public final SparkMax driveMotor;
        private final SparkMax turningMotor;

        private final RelativeEncoder driveEncoder;
        private final RelativeEncoder turningEncoder;
        private final CANcoder absoluteEncoder;

        // i i i ffffffffffffffffffff
        // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.18359,
        // 3.0413);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.00847, 0.27927, 0.09857);

        // add values later
        private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.13943, 0.39686, 0.015295);

        private SparkClosedLoopController pidController;

        private final TrapezoidProfile.Constraints turnConstraints = new TrapezoidProfile.Constraints(
                        Float.POSITIVE_INFINITY,
                        200 * 2);

        public double getAbsRad() {
                return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2;
        }

        public SwerveModule(
                        int driveMotorID,
                        int turningMotorID,
                        int turningEncoderID,
                        boolean driveMotorInverted,
                        boolean turningMotorInverted,
                        double magnetOffset) {
                driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
                turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

                absoluteEncoder = new CANcoder(turningEncoderID);

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

                driveMotor.setControlFramePeriodMs(100);
                driveMotor.setControlFramePeriodMs(20);
                driveMotor.setControlFramePeriodMs(20);

                turningMotor.setControlFramePeriodMs(10);
                turningMotor.setControlFramePeriodMs(20);
                turningMotor.setControlFramePeriodMs(50);

                driveEncoder = driveMotor.getEncoder();
                // TODO: FIX CONSTANTS
                double drivePositionConversionFactor = Math.PI * Constants.ModuleType.getWheelDiameter()
                                * Constants.ModuleType.getDriveReduction();

                SparkMaxConfig driveConfig = new SparkMaxConfig();
                driveConfig
                                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                                .inverted(driveMotorInverted)
                                .smartCurrentLimit(50);
                driveConfig.encoder
                                .positionConversionFactor(drivePositionConversionFactor)
                                .velocityConversionFactor(drivePositionConversionFactor / 60);
                driveConfig.closedLoop
                                .pid(1.0, 0, 0.1)
                                .outputRange(-1, 1);
                driveMotor.configure(driveConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                                SparkBase.PersistMode.kNoPersistParameters);

                turningEncoder = turningMotor.getEncoder();
                
                double turningPositionConversionFactor = Math
                                .toRadians(Constants.ModuleConstants.TurningEncoderDegreesPerPulse);

                // turn motor configurations
                SparkMaxConfig turnConfig = new SparkMaxConfig();
                turnConfig
                                .inverted(driveMotorInverted)
                                .idleMode(SparkMaxConfig.IdleMode.kCoast)
                                .smartCurrentLimit(20);
                turnConfig.encoder
                                .positionConversionFactor(turningPositionConversionFactor)
                                .velocityConversionFactor(turningPositionConversionFactor / 60);
                turnConfig.closedLoop
                                .pid(1.0, 0.0, 0.1)
                                .outputRange(-1, 1);
                turningMotor.configure(turnConfig, SparkMax.ResetMode.kResetSafeParameters,
                                SparkMax.PersistMode.kNoPersistParameters);

                turningEncoder.setPosition(getAbsRad());

                pidController = turningMotor.getClosedLoopController();
        }

        public SwerveModuleState getState() {
                return new SwerveModuleState(
                                driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
        }

        public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(
                                driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
        }

        public void setDesiredState(SwerveModuleState desiredState) {
                // Optimize the reference state to avoid spinning further than 90 degrees
                SwerveModuleState state = desiredState;

                state.speedMetersPerSecond *= desiredState.angle.minus(getState().angle).getCos();

                var fff = feedforward.calculate(state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                driveMotor.setVoltage(
                                ((state.speedMetersPerSecond / Constants.DriveConstants.MaxVelocityMetersPerSecond)
                                                * 12) + fff);

                pidController.setReference(state.angle.getRadians(), ControlType.kPosition);
        }

        public SwerveModuleState optimizeModuleState(SwerveModuleState state) {
                state.optimize(Rotation2d.fromRadians(turningEncoder.getPosition()));

                double currentAngleRadiansMod = turningEncoder.getPosition() % (2.0 * Math.PI);
                if (currentAngleRadiansMod < 0.0) {
                        currentAngleRadiansMod += 2.0 * Math.PI;
                }

                // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
                // that
                double adjustedReferenceAngleRadians = state.angle.getRadians() + turningEncoder.getPosition()
                                - currentAngleRadiansMod;
                if (state.angle.getRadians() - currentAngleRadiansMod > Math.PI) {
                        adjustedReferenceAngleRadians -= 2.0 * Math.PI;
                } else if (state.angle.getRadians() - currentAngleRadiansMod < -Math.PI) {
                        adjustedReferenceAngleRadians += 2.0 * Math.PI;
                }

                return new SwerveModuleState(state.speedMetersPerSecond,
                                Rotation2d.fromRadians(adjustedReferenceAngleRadians));
        }

        public void resetEncoders() {
                driveEncoder.setPosition(0);
        }
}

// but im a CounterClockwise_Positive
// im a weirdoOoooo
// what the hell am i doing here
// i dont belong here
// //oOoO
// shEEEeeeees
// rUnning out of TIIIMmeeee
// shEEessss
// rUUning out of TimETIMETIMETIMEEEEEEEEEeeeee
