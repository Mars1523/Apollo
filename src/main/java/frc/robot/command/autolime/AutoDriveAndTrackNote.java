package frc.robot.command.autolime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.SwerveSubsystem;

public class AutoDriveAndTrackNote extends Command {

    private SwerveSubsystem swerveSub;
    private double goalDistance;
    private double speed;
    private Translation2d startPosition;
    SlewRateLimiter slew = new SlewRateLimiter(0.2);

    MedianFilter horizontalFilter = new MedianFilter(3);
    private ProfiledPIDController rotationPid;
    private int time;
    final double getHorizontalOffset() {
        return horizontalFilter.calculate(LimelightHelpers.getTX("limelight-front"));
    }

    public AutoDriveAndTrackNote(SwerveSubsystem swerveSub, double goalDistance, double speed) {

        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.goalDistance = goalDistance;
        this.speed = speed;
        // this.slew = slew;
        rotationPid = new ProfiledPIDController(0.03, 0, 0,
        // horizontalPid = new ProfiledPIDController(kuh*.6, 0, tuh*.125,
        new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3/1.5));

    }

    public AutoDriveAndTrackNote(Intake intakeSub) {
        //TODO Auto-generated constructor stub
    }

    @Override
    public void initialize() {
        startPosition = swerveSub.getPose().getTranslation();
        slew.reset(0);
        time = 0;
    }

SlewRateLimiter turnLimit = new SlewRateLimiter(1);
    @Override
    public void execute() {
        // drive forqard
        var autoRotPercent = 0.0;
        if (LimelightHelpers.getTV("limelight-front")) {
            autoRotPercent = rotationPid.calculate(getHorizontalOffset());
            autoRotPercent = MathUtil.clamp(autoRotPercent, -DriveConstants.MaxVelocityMetersPerSecond/5, DriveConstants.MaxVelocityMetersPerSecond/5);

            autoRotPercent= turnLimit.calculate(autoRotPercent/DriveConstants.MaxAngularVelocityRadiansPerSecond);
        }

        NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Tune/Commanded Speed").setDouble(speed);
        var ss=slew.calculate(speed);
        NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Tune/Limited Speed").setDouble(ss);
        swerveSub.drive(ss, 0, autoRotPercent, false);
        // hird

    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.drive(0, 0, 0, false, 0, 0);
    }

    @Override
    public boolean isFinished() {

        double dist = swerveSub.getPose().getTranslation().getDistance(startPosition);
        if (goalDistance < dist) {
            return true;

        } else {
            return false;
        }
        // i finally get it now
        // goal distance is how far we want the robot to go
        // dist is how far we currently are away from the starting pos
        // if how far away we currently are is greater than how far away we want to be
        // the auto is finished
        // :DDDD
    }
}