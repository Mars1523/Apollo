package frc.robot.subsytems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class SwerveModule {

        // add motor model later

        // defines motors for a singular module

        private final CANSparkMax driveMotor;
        private final CANSparkMax turningMotor;

        // add encoder model later

        // defines encoders for a singular module

        private final RelativeEncoder driveEncoder;
        private final RelativeEncoder turningEncoder;
        private final CANCoder absoluteEncoder;

        // pid drive controller somehow
        // add tuning values

        private final PIDController drivePIDController = new PIDController(.1, 0, 0);

        // pid turning trapezoidal

        private final PIDController turningPIDController = new PIDController(1.0 / (Math.PI / 2), 0, .0);

        // add values later

        // private final SimpleMotorFeedforward driveFeedforward = new
        // SimpleMotorFeedforward(s, v);
        private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.13943, 0.39686, 0.015295);

        private SparkMaxPIDController pidController;

        private final TrapezoidProfile.Constraints turnConstraints = new TrapezoidProfile.Constraints(
                        Float.POSITIVE_INFINITY,
                        200 * 2);

        public SwerveModule(
                        int driveMotorID,
                        int turningMotorID,
                        int turningEncoderID,
                        boolean driveEncoderReversed,
                        boolean turningEncoderReversed,
                        Rotation2d magnetOffset) {

                // add motor name
                // makes it so you can define motor channels for the modules in subsystem
                driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
                turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
                turningMotor.restoreFactoryDefaults();

                driveMotor.setInverted(driveEncoderReversed);
                turningMotor.setInverted(turningEncoderReversed);

                // add encoder name
                // makes it so you can define encoder channels for the modules in subsystem

                absoluteEncoder = new CANCoder(turningEncoderID);

                turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

                var config = new CANCoderConfiguration();
                config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
                config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                config.sensorTimeBase = SensorTimeBase.PerSecond;
                config.magnetOffsetDegrees = magnetOffset.getDegrees();
                config.sensorDirection = !turningEncoderReversed;
                // absoluteEncoder.configAllSettings(config, 250);
                // absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100,
                // 250);

                absoluteEncoder.configAllSettings(config, 250);
                absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250);

                // #region Motor controller setup
                driveMotor.setInverted(driveEncoderReversed);
                turningMotor.setInverted(turningEncoderReversed);

                turningMotor.setSmartCurrentLimit(20);
                driveMotor.setSmartCurrentLimit(80);

                driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
                driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
                driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
                // Set neutral mode
                driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

                turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
                turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
                turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);
                // Set neutral mode
                turningMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
                // #endregion

                driveEncoder = driveMotor.getEncoder();
                // TODO: FIX CONSTANTS
                double positionConversionFactor = Math.PI * Constants.ModuleType.getWheelDiameter()
                                * Constants.ModuleType.getDriveReduction();
                driveEncoder.setPositionConversionFactor(positionConversionFactor);
                driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60);

                turningEncoder = turningMotor.getEncoder();

                turningEncoder
                                .setPositionConversionFactor(Math
                                                .toRadians(Constants.ModuleConstants.TurningEncoderDegreesPerPulse));
                turningEncoder
                                .setVelocityConversionFactor(
                                                Math.toRadians(Constants.ModuleConstants.TurningEncoderDegreesPerPulse)
                                                                / 60);
                turningEncoder.setPosition(
                                Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition()));

                turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

                pidController = turningMotor.getPIDController();
                pidController.setP(1.0);
                pidController.setI(0.0);
                pidController.setD(0.1);
                // pidController.setFF(1.534);
                // pidController.setOutputRange(-.5, .5);

                // Shuffleboard.getTab("Debug").addDouble("Turn Output Raw", () ->
                // m_turningMotor.get());
                // Shuffleboard.getTab("Debug").addDouble("Drive Output Raw", () ->
                // m_driveMotor.get());
                Shuffleboard.getTab("Debug")
                                .addDouble("Measured Abs rotation" + turningEncoderID,
                                                () -> Units.rotationsToDegrees(absoluteEncoder.getAbsolutePosition()));
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
                SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                                new Rotation2d(turningEncoder.getPosition()));

                final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(),
                                state.speedMetersPerSecond);

                final double turnOutput = turningPIDController.calculate(turningEncoder.getPosition(),
                                state.angle.getRadians());

                // sets the motors to the calculated output
                driveMotor.set(driveOutput);
                turningMotor.set(turnOutput);
        }

        public void resetEncoders() {
                driveEncoder.setPosition(0);
                turningEncoder.setPosition(0);
        }
}
