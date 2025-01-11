package frc.robot.subsytems;

import javax.swing.plaf.nimbus.State;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.BaseStatusSignal;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    public Shooter() {
        Shuffleboard.getTab("Debug").addDouble("MotorSpeed", this::getSpeedOne);
        Shuffleboard.getTab("Debug").addDouble("MotorSpeed2", this::getSpeedTwo);        // shooterOne.configVoltageCompSaturation(12);
        // shooterTwo.configVoltageCompSaturation(12);
    }

    // motor
    // to be quite honest i dunnoi what goe s to which motor controller
    private TalonSRX shooterOne = new TalonSRX(10);
    private TalonSRX shooterTwo = new TalonSRX(11);
    // private final TalonSRX ampDeflector = new TalonSRX(0);
    // private final CANcoder deflectorEncoder = new CANcoder(0);
    //private double ampSet;
    // shooterOne.setUpdateFrequency(200);
    // set the update frequency

    ProfiledPIDController deflectorPID = new ProfiledPIDController(
            1.1,
            0,
            0,
            new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 2));

    // moter
    public void makeItGoBackwards(){
        shooterOne.set(TalonSRXControlMode.PercentOutput, 0.27);
        shooterTwo.set(TalonSRXControlMode.PercentOutput, -0.27);
    }

    public void shootAmp() {

        // deflectorEncoder.getAbsolutePosition().setUpdateFrequency(100, 250);
        deflectorPID.setGoal(0.5);
        // goal is wherever the motor is when it is in the right position to deflect the
        // note

        // deflectorPID increments the deflector motor until the encoder value = the
        // goal
        // ampDeflector.set(TalonSRXControlMode.PercentOutput,
        // deflectorPID.calculate(ampEncoder));
        // ampDeflector.set(TalonSRXControlMode.PercentOutput, ampSet);
        shooterOne.set(TalonSRXControlMode.PercentOutput, -.5);
        shooterTwo.set(TalonSRXControlMode.PercentOutput, .5);
        // whatever
    }

    public void shootSpeaker() {
        // etc
        // sets deflect
        deflectorPID.setGoal(0);

        // deflectorPID increments the deflector motor until the encoder value = the
        // goal
        // ampDeflector.set(TalonSRXControlMode.PercentOutput,
        // deflectorPID.calculate(ampEncoder));
        // ampDeflector.set(TalonSRXControlMode.PercentOutput, ampSet);
        shooterOne.set(TalonSRXControlMode.PercentOutput, -.9);
        shooterTwo.set(TalonSRXControlMode.PercentOutput, .9);
    }

    public void reverseShooters() {
        shooterOne.set (TalonSRXControlMode.PercentOutput, .5);
        shooterTwo.set(TalonSRXControlMode.PercentOutput, -0.5);
    }

    public boolean isToAmpSpeed() {
        if (shooterOne.getSelectedSensorVelocity() > 1500 && shooterTwo.getSelectedSensorVelocity() > 1500) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isToSpeakerSpeed() {
        if (Math.abs(getSpeedTwo()) > 4100) {
            return true;
        } else {
            return false;//hello
        }
    }

    public void turnOff() {

        shooterOne.set(TalonSRXControlMode.PercentOutput, .0);
        shooterTwo.set(TalonSRXControlMode.PercentOutput, .0);

    }

    // public double getMotorSpeed() {
    //     return shooterOne.();
    //     // return motor speed
    // }

    public double getRotations() {
        // return rotations of the shooter wheel to use in auto
        double rots;
        rots = shooterOne.getSelectedSensorPosition();
        return rots;

    }

    public double getSpeedOne() {
        double stuff = (shooterOne.getSelectedSensorVelocity() * 600 / 4096 / 3);
        return stuff;
    }

    public double getSpeedTwo() {
        double stuff = (shooterTwo.getSelectedSensorVelocity() * 600 / 4096 / 3);
        return stuff;
    }

    @Override
    public void periodic() {
        // ampSet =
        // deflectorPID.calculate(deflectorEncoder.getAbsolutePosition().getValueAsDouble());

        // m
    }

}
