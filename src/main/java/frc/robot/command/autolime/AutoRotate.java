package frc.robot.command.autolime;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.SwerveSubsystem;

public class AutoRotate extends Command {

private SwerveSubsystem swerveSub;
private double goalYaw;
private double turnSpeed;
private Rotation2d startingYaw;

// private boolean inverted;

    public AutoRotate(SwerveSubsystem swerveSub, double goalYaw, double turnSpeed){

        addRequirements(swerveSub);

        this.swerveSub = swerveSub;
        this.turnSpeed = turnSpeed;
        this.goalYaw = goalYaw;
        if(goalYaw < 0){
            this.turnSpeed = -turnSpeed;
        }
    }

    @Override
    public void initialize(){
        startingYaw = swerveSub.getRotation(); 
        if(goalYaw < 0){
            turnSpeed = -turnSpeed;
        }
    }
    @Override
    public void execute(){
        swerveSub.drive(0, 0, turnSpeed, false, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.drive(0, 0, 0, false, 0, 0);
    }


    @Override
    public boolean isFinished(){
        if (Math.abs(goalYaw) < Math.abs(swerveSub.getRotation().minus(startingYaw).getDegrees())) {
            return true;
        } else {
            return false;
        }

        // double tempGY;
        // double tempCY;
        // double currentYaw = swerveSub.getYaw();
        // if(goalYaw < 0){
        //     tempGY = -goalYaw;
        // }
        // else{
        //     tempGY = goalYaw;
        // }
        // if(currentYaw < 0){
        //     tempCY = -currentYaw;
        // }
        // else{
        //     tempCY = currentYaw;
        // }
        // //yeah i dunno
        // // -90 -> 90
        // // -97 -> 97 
        // // 60 -> 60
        // // 97 > 90 -> true 
        
        // if (tempGY <= tempCY){
        //     return true;
        // }
        // else{
        //     return false;
        // }


    }
}