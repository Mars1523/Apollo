package frc.robot.subsytems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class SwerveModule {

        // add motor model later

        // defines motors for a singular module

        public final SparkMax driveMotor;
        private final SparkMax turningMotor;

        // add encoder model later

        // defines encoders for a singular module

        private final RelativeEncoder driveEncoder;
        private final RelativeEncoder turningEncoder;
        private final CANcoder absoluteEncoder;

        // pid drive controller somehow
        // add tuning values

        //private final PIDController drivePIDController = new PIDController(.1, 0, 0);

        // i i i ffffffffffffffffffff
        //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.18359, 3.0413);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.00847, 0.27927, 0.09857);
        // pid turning trapezoidal

        // private final PIDController turningPIDController = new PIDController(1.0 /
        // (Math.PI / 2), 0, .0);

        // add values later

        // private final SimpleMotorFeedforward driveFeedforward = new
        // SimpleMotorFeedforward(s, v);
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
                // add motor name
                // makes it so you can define motor channels for the modules in subsystem
                driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
                turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);
                //turningMotor.restoreFactoryDefaults();

                //driveMotor.setInverted(driveMotorInverted);
                //turningMotor.setInverted(turningMotorInverted);

                // add encoder name
                // makes it so you can define encoder channels for the modules in subsystem

                absoluteEncoder = new CANcoder(turningEncoderID);

                // turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

                var config = new MagnetSensorConfigs();
                //config.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorDiscontinuityPoint.Signed_PlusMinusHalf;
                config.withAbsoluteSensorDiscontinuityPoint(0.5);
                config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

                // absoluteEncoder.configAllSettings(config, 250);
                // absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100,
                // 250);
                if (magnetOffset <= 0) {
                        config.MagnetOffset = (-magnetOffset) - .5;
                } else {
                        config.MagnetOffset = (-magnetOffset) + .5;
                }

                absoluteEncoder.getConfigurator().apply(config);
                absoluteEncoder.getAbsolutePosition().setUpdateFrequency(100, 250);

                // #region Motor controller setup
                // driveMotor.setInverted(driveMotorInverted);
                // turningMotor.setInverted(turningMotorInverted);

                //turningMotor.setSmartCurrentLimit(20);
                //driveMotor.setSmartCurrentLimit(50);

                driveMotor.setControlFramePeriodMs(100);
                driveMotor.setControlFramePeriodMs(20);
                driveMotor.setControlFramePeriodMs(20);
                // Set neutral mode

                //driveMotor.setIdleMode(SparkMax.kBrake);
                

                turningMotor.setControlFramePeriodMs(10);
                turningMotor.setControlFramePeriodMs(20);
                turningMotor.setControlFramePeriodMs(50);
                // Set neutral mode
                //turningMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
                
                                
                // #endregion

                driveEncoder = driveMotor.getEncoder();
                // TODO: FIX CONSTANTS
                double drivePositionConversionFactor = Math.PI * Constants.ModuleType.getWheelDiameter()
                                * Constants.ModuleType.getDriveReduction();
                //driveEncoder.setPositionConversionFactor(positionConversionFactor);
                //driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60);

                SparkMaxConfig driveConfig = new SparkMaxConfig();
                driveConfig
                        .idleMode(SparkBaseConfig.IdleMode.kBrake)
                        .inverted(driveMotorInverted)
                        .smartCurrentLimit(50);
                driveConfig.encoder
                        .positionConversionFactor(drivePositionConversionFactor)
                        .velocityConversionFactor(drivePositionConversionFactor/60);
                driveMotor.configure(driveConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

                turningEncoder = turningMotor.getEncoder();

                double turningPositionConversionFactor = Math.toRadians(Constants.ModuleConstants.TurningEncoderDegreesPerPulse);

                
                //turn motor configurations
                SparkMaxConfig turnConfig = new SparkMaxConfig();
                turnConfig
                        .inverted(driveMotorInverted)
                        .idleMode(SparkBaseConfig.IdleMode.kCoast)
                        .smartCurrentLimit(20);
                turnConfig.encoder
                        .positionConversionFactor(turningPositionConversionFactor)
                        .velocityConversionFactor(turningPositionConversionFactor/60);
                turnConfig.closedLoop
                        .pid(1.0,0.0, 0.1)
                        .outputRange(-1, 1);
                turningMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
                
                // turningEncoder
                //                 .setPositionConversionFactor(Math
                //                                 .toRadians(Constants.ModuleConstants.TurningEncoderDegreesPerPulse));
                // turningEncoder
                //                 .setVelocityConversionFactor(
                //                                 Math.toRadians(Constants.ModuleConstants.TurningEncoderDegreesPerPulse)
                //                                                 / 60);
                turningEncoder.setPosition(getAbsRad());

                SparkClosedLoopController turningPID = turningMotor.getClosedLoopController();
                // turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

                // pidController = turningMotor.getPIDController();
                // pidController.setP(1.0);
                // pidController.setI(0.0);
                // pidController.setD(0.1);
                // // pidController.setFF(1.534);
                // pidController.setOutputRange(-1, 1);

                // Shuffleboard.getTab("Debug").addDouble("Turn Output Raw", () ->
                // m_turningMotor.get());
                // Shuffleboard.getTab("Debug").addDouble("Drive Output Raw", () ->
                // m_driveMotor.get());
                Shuffleboard.getTab("Debug")
                                .addDouble("Measured Abs rotation" + turningEncoderID,
                                                () -> getAbsRad());
                // Shuffleboard.getTab("Debug").addDouble("Integrated encoder", () ->
                // m_integratedTurningEncoder.getPosition());
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
                // SwerveModuleState state = optimizeModuleState(desiredState);
                SwerveModuleState state = desiredState;

                state.speedMetersPerSecond *= desiredState.angle.minus(getState().angle).getCos();

                // Math.cos(desiredState.angle - getState().angle);

                var fff = feedforward.calculate(state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                driveMotor.setVoltage(
                                ((state.speedMetersPerSecond / Constants.DriveConstants.MaxVelocityMetersPerSecond)
                                                * 12) + fff);

                pidController.setReference(state.angle.getRadians(), ControlType.kPosition);
        }

        public SwerveModuleState optimizeModuleState(SwerveModuleState rawState) {
                var optimizedState = SwerveModuleState.optimize(rawState,
                                Rotation2d.fromRadians(turningEncoder.getPosition()));

                double currentAngleRadiansMod = turningEncoder.getPosition() % (2.0 * Math.PI);
                if (currentAngleRadiansMod < 0.0) {
                        currentAngleRadiansMod += 2.0 * Math.PI;
                }

                // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
                // that
                double adjustedReferenceAngleRadians = optimizedState.angle.getRadians() + turningEncoder.getPosition()
                                - currentAngleRadiansMod;
                if (optimizedState.angle.getRadians() - currentAngleRadiansMod > Math.PI) {
                        adjustedReferenceAngleRadians -= 2.0 * Math.PI;
                } else if (optimizedState.angle.getRadians() - currentAngleRadiansMod < -Math.PI) {
                        adjustedReferenceAngleRadians += 2.0 * Math.PI;
                }

                // pidController.setReference(0,ControlType.kPosition);
                // pidController.setReference(state.angle.getRadians(), ControlType.kPosition);

                return new SwerveModuleState(optimizedState.speedMetersPerSecond,
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
