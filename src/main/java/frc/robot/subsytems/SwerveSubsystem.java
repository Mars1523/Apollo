package frc.robot.subsytems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

//add motor channel numbers later
public class SwerveSubsystem extends SubsystemBase {

        private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter rotRateLimiter = new SlewRateLimiter(2);

        private final SwerveModule fLSwerve = new SwerveModule(15, 14, 20, true, true, -0.137);
        private final SwerveModule fRSwerve = new SwerveModule(13, 12, 19, true, true, 0);
        private final SwerveModule bLSwerve = new SwerveModule(17, 16, 21, true, true, 0.172);
        private final SwerveModule bRSwerve = new SwerveModule(11, 10, 18, true, true, -0.429);

        private static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

        public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative, double a,
                        double b) {
                drive(xPercent, yPercent, rotPercent, fieldRelative);
        }

        public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
                var xSpeed = xRateLimiter.calculate(xPercent) * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var ySpeed = yRateLimiter.calculate(yPercent) * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var rot = rotRateLimiter.calculate(rotPercent)
                                * Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond;

                ChassisSpeeds chasSpeed = fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot);

                var swerveModuleStates2 = DriveConstants.kinematics.toSwerveModuleStates(
                                ChassisSpeeds.discretize(chasSpeed, 0.2));

                driveStates(swerveModuleStates2);
        }

        public Rotation2d yawOffset = new Rotation2d();

        SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
                        DriveConstants.kinematics,
                        getRotation(),
                        new SwerveModulePosition[] { fLSwerve.getPosition(), fRSwerve.getPosition(),
                                        bLSwerve.getPosition(), bRSwerve.getPosition() },
                        new Pose2d(1, 1, new Rotation2d()));

        public Rotation2d getRotation() {
                return gyro.getRotation2d().minus(yawOffset);
        }

        public void zeroYaw() {
                if (gyro.getRotation2d() != null) {
                        yawOffset = gyro.getRotation2d();
                }
        }

        public void botposewithapriltag() {
                var aprilRotation = LimelightHelpers.getBotPose2d("limelight-back").getRotation();
                if (aprilRotation.getDegrees() == 0) {
                        return;
                }
                yawOffset = gyro.getRotation2d().minus(aprilRotation);
        }
        
        boolean seenMT = false;
        void mtVision() {
                boolean useMegaTag2 = false; // set to false to use MegaTag1
                boolean doRejectUpdate = false;
                if (useMegaTag2 == false) {
                        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue("limelight-back");

                        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                                if (mt1.rawFiducials[0] == null) {
                                        return;
                                }
                                if (mt1.rawFiducials[0].ambiguity > .7) {
                                        doRejectUpdate = true;
                                }
                                if (mt1.rawFiducials[0].distToCamera > 3) {
                                        doRejectUpdate = true;
                                }
                        }
                        if (mt1.tagCount == 0) {
                                doRejectUpdate = true;
                        }

                        if (!doRejectUpdate) {
                                seenMT = true;
                                estimator.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, 5));
                                estimator.addVisionMeasurement(
                                                mt1.pose,
                                                mt1.timestampSeconds);
                        }
                } else if (useMegaTag2 == true) {
                        LimelightHelpers.SetRobotOrientation("limelight-back",
                                        estimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
                        if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per
                        // second, ignore vision updates
                        {
                                doRejectUpdate = true;
                        }
                        if (mt2.tagCount == 0) {
                                doRejectUpdate = true;
                        }
                        if (!doRejectUpdate) {
                                seenMT = true;
                                estimator.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, 5));
                                estimator.addVisionMeasurement(
                                                mt2.pose,
                                                mt2.timestampSeconds);

                        }
                }
        }

        @Override
        public void periodic() {
                mtVision();

                // TODO Auto-generated method stub
                estimator.update(
                                getRotation(),
                                new SwerveModulePosition[] {
                                                fLSwerve.getPosition(),
                                                fRSwerve.getPosition(),
                                                bLSwerve.getPosition(),
                                                bRSwerve.getPosition()
                                }

                );

                field.setRobotPose(getPose());
        }

        public Pose2d getPose() {
                return estimator.getEstimatedPosition();
        }

        public void resetOmetry(Pose2d pose) {
                estimator.resetPosition(
                                getRotation(),
                                new SwerveModulePosition[] {
                                                fLSwerve.getPosition(),
                                                fRSwerve.getPosition(),
                                                bLSwerve.getPosition(),
                                                bRSwerve.getPosition()
                                },
                                pose);
        }

        public SwerveModuleState[] getModuleStates() {
                SwerveModuleState[] states = {
                                fLSwerve.getState(),
                                fRSwerve.getState(),
                                bLSwerve.getState(),
                                bRSwerve.getState(),
                };
                return states;
        }

        public ChassisSpeeds getSpeeds() {
                return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
        }

        public void driveStates(SwerveModuleState[] swerveModuleStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                                Constants.DriveConstants.MaxVelocityMetersPerSecond);

                SwerveModuleState[] optimizedSwerveModuleStates = {
                                fLSwerve.optimizeModuleState(swerveModuleStates[0]),
                                fRSwerve.optimizeModuleState(swerveModuleStates[1]),
                                bLSwerve.optimizeModuleState(swerveModuleStates[2]),
                                bRSwerve.optimizeModuleState(swerveModuleStates[3]),
                };

                fLSwerve.setDesiredState(optimizedSwerveModuleStates[0]);
                fRSwerve.setDesiredState(optimizedSwerveModuleStates[1]);
                bLSwerve.setDesiredState(optimizedSwerveModuleStates[2]);
                bRSwerve.setDesiredState(optimizedSwerveModuleStates[3]);
        }

        public SwerveSubsystem() {
                // PathPlanner config
                RobotConfig config;
                try {
                        config = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                        e.printStackTrace();
                        throw new RuntimeException(e);
                }
                // Configure AutoBuilder last
                AutoBuilder.configure(
                                this::getPose, // Robot pose supplier
                                this::resetOmetry, // Method to reset odometry (will be called if your auto has a
                                                   // starting pose)
                                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                (speeds, feedforwards) -> {
                                        var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
                                                        ChassisSpeeds.discretize(speeds, .02));
                                        driveStates(swerveModuleStates);
                                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live
                                                                // in your
                                                                // Constants class
                                                new PIDConstants(1, 0.0, .0), // Translation PID constants
                                                new PIDConstants(1, 0.0, 0.0) // Rotation PID constants
                                ),
                                config,
                                () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red
                                        // alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                        var alliance = DriverStation.getAlliance();
                                        if (alliance.isPresent()) {
                                                return alliance.get() == DriverStation.Alliance.Red;
                                        }
                                        return false;
                                },
                                this // Reference to this subsystem to set requirements
                );
                Shuffleboard.getTab("Debug").addDouble("drive velocity unfiltered",
                                () -> fLSwerve.driveMotor.getEncoder().getVelocity());
                Shuffleboard.getTab("Debug").addDouble("drive position unfiltered",
                                () -> fLSwerve.driveMotor.getEncoder().getPosition());
                Shuffleboard.getTab("Debug").addDouble("robot angle from april tags",
                                () -> LimelightHelpers.getBotPose2d("limelight-back").getRotation().getDegrees());
                Shuffleboard.getTab("Debug").addDouble("robot angle from navx",
                                () -> gyro.getRotation2d().getDegrees());
                Shuffleboard.getTab("Debug").addDouble("yaw offset", () -> yawOffset.getDegrees());

                Shuffleboard.getTab("Drive").add(field);
        }

        Field2d field = new Field2d();
}