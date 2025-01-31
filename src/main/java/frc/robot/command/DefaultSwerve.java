package frc.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsytems.*;

public class DefaultSwerve extends Command {

    private Joystick joy;
    private SwerveSubsystem swerveSub;
    Boolean slow = false;

    public DefaultSwerve(Joystick joy, SwerveSubsystem swerveSub) {

        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.joy = joy;
    }

    private double signedPow(double a, double pow){
        return Math.copySign(Math.pow(a, pow), a);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // swerve stuff goes here
        // xspeed is xbox controller left joystick yspeed is also left joystick and
        // rotation is right joystick

        // adding deadbands

        var xSpeed = (MathUtil.applyDeadband(-joy.getY(), 0.1));
        var ySpeed = (MathUtil.applyDeadband(-joy.getX(), 0.1));
        var rot = (MathUtil.applyDeadband(-joy.getTwist(), 0.1));

        xSpeed = signedPow(xSpeed, 2);
        ySpeed = signedPow(ySpeed, 2);
        rot = signedPow(rot, 2);

        System.out.println(xSpeed);
        if (!joy.getTrigger()) {
            xSpeed *= 0.5;
            ySpeed *= 0.5;
            rot *= 0.4;
        } else {
            rot *= 0.6 ;
            xSpeed *= 0.8;
            ySpeed *= 0.8;
        }

        if (joy.getRawButton(7)) {
            xSpeed *= 0.75;
            ySpeed *= 0.75;
            rot *= 0.25;
        }
  
        if (joy.getRawButton(12)) {
            swerveSub.zeroYaw();
        }

        swerveSub.drive(xSpeed, ySpeed, rot, !joy.getRawButton(2));

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
