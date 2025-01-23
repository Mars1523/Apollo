// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.sds.ModuleConfiguration;
import frc.robot.sds.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class AutoConstants {
    public static double kPYController = 3;
    public static double kPXController = 3;
    public static double kPThetaController = 1;
    public static double kMaxSpeedMetersPerSecond = DriveConstants.MaxVelocityMetersPerSecond * .50;
    public static double kMaxAccelerationMetersPerSecondSquared = 4;
    public static Constraints kThetaControllerConstraints = new Constraints(
        DriveConstants.MaxAngularVelocityRadiansPerSecond * .5, (Math.PI * 2) / 2);
    
    public static PathConstraints constantConstraints = new PathConstraints(2.5,2,2.5*Math.PI,3*Math.PI);

  }

  public static final ModuleConfiguration ModuleType = SdsModuleConfigurations.MK4I_L1;

  public static final class DriveConstants {
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    // Distance between left and right wheels
    public static final double kTrackWidthMeters = 0.6223/2;
    // Distance between front and back wheels
    public static final double kTrackBaseMeters = 0.40004999999999996/2;

    private static final Translation2d kFrontLeftLocation = new Translation2d(kTrackBaseMeters, kTrackWidthMeters);
    private static final Translation2d kFrontRightLocation = new Translation2d(kTrackBaseMeters, -kTrackWidthMeters);
    private static final Translation2d kBackLeftLocation = new Translation2d(-kTrackBaseMeters, kTrackWidthMeters);
    private static final Translation2d kBackRightLocation = new Translation2d(-kTrackBaseMeters, -kTrackWidthMeters);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double kNeoFreeSpinRpm = 5676;
    public static final double MaxVelocityMetersPerSecond = (kNeoFreeSpinRpm / 60.0) *
        ModuleType.getDriveReduction() *
        ModuleType.getWheelDiameter() * Math.PI;

    public static final double MaxAngularVelocityRadiansPerSecond = MaxVelocityMetersPerSecond
        / Math.hypot(kTrackWidthMeters / 2, kTrackBaseMeters / 2) *.75;

  }

  public static final class ModuleConstants {

    public static final double MaxModuleAngularSpeedRadiansPerSecond = 2 *
        Math.PI;
    public static final double MaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    // public static final int kEncoderCPR = 1024;
    // public static final double kWheelDiameterMeters = 0.15;
    // public static final double kDriveEncoderDistancePerPulse =
    // // Assumes the encoders are directly mounted on the wheel shafts
    // (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // public static final double kTurningEncoderDistancePerPulse =
    // // Assumes the encoders are on a 1:1 reduction with the module shaft.
    // (2 * Math.PI) / (double) kEncoderCPR;
    public static final double TurningEncoderDegreesPerPulse = Math
        .toDegrees(2. * Math.PI * ModuleType.getSteerReduction());

    // public static final double kPModuleTurningController = 1;

    // public static final double kPModuleDriveController = 1;
  }

  


}