package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    VictorSPX intakeOne = new VictorSPX(8);
    VictorSPX intakeTwo = new VictorSPX(9);
    TalonSRX transport = new TalonSRX(12);
    // private PIDController intakeP

    // laser
    private LaserCan laser = new LaserCan(44);

    private Shooter shooterSub;

    // motors that first grab the note under the bumber
    // kind of like beatle jaws
    public Intake(Shooter shooterSub) {

        Shuffleboard.getTab("Drive").addBoolean("hasNote", this::hasNote);
        this.shooterSub = shooterSub;

        intakeTwo.setInverted(true);
    }

    // boolena
    private boolean isTaking = false;

    public boolean get() {
        return isTaking;
    }

    public void smartIntake() {
        isTaking = true;
    }

    public void stopSmIntake() {
        isTaking = false;
    }

    private boolean isFeeding = false;

    public boolean getFeeding() {
        return isFeeding;
    }

    public void feedOn() {
        isFeeding = true;
    }

    public void feedOff() {
        isFeeding = false;
    }

    public void eat() {

        intakeOne.set(ControlMode.PercentOutput, 0.2);
        intakeTwo.set(ControlMode.PercentOutput, 0.2);
        transport.set(ControlMode.PercentOutput, 0.2);
    }

    public void sing() {
        // set all the values of suck to negative???? maybe???? idk???
        intakeOne.set(ControlMode.PercentOutput, -0.5);
        intakeTwo.set(ControlMode.PercentOutput, 0.5);
        transport.set(ControlMode.PercentOutput, -0.5);
    }

    public void intakeOff() {
        intakeOne.set(ControlMode.PercentOutput, 0);
        intakeTwo.set(ControlMode.PercentOutput, 0);
        transport.set(ControlMode.PercentOutput, 0);
    }

    public void feedToShooter() {
        intakeOne.set(ControlMode.PercentOutput, 0.2);
    }

    public void feedBackwards() {
        transport.set(ControlMode.PercentOutput, -0.2);
        intakeTwo.set(ControlMode.PercentOutput, .2);
    }

    public void printshtuff() {
        LaserCan.Measurement m = laser.getMeasurement();
        System.out
                .println("mm:" + m.distance_mm + " valid " + (m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT));
    }

    public boolean hasNote() {
        LaserCan.Measurement m = laser.getMeasurement();
        if (m != null && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && m.distance_mm < 245) {
            return true;
        } else {
            return false;
        }
    }

    public boolean doesntHaveNote() {
        return !hasNote();
    }

    private PIDController notePID = new PIDController(.004, 0, 0);
    MedianFilter laserFilter = new MedianFilter(5);

    @Override
    public void periodic() {
        // intakeTwo.set(ControlMode.PercentOutput, .1);

        LaserCan.Measurement measurement = laser.getMeasurement();

        if (isTaking) {
            intakeOne.set(ControlMode.PercentOutput, 0.6);
            intakeTwo.set(ControlMode.PercentOutput, 0.85);
            // if (measurement != null)
            // System.out.println("measurement:" + measurement.status + " valid "
            // + (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT));
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                if (measurement.distance_mm < 300) {
                    intakeOne.set(ControlMode.PercentOutput, .0);
                    intakeTwo.set(ControlMode.PercentOutput, .5);
                    // transport.set(ControlMode.PercentOutput, 0.0);
                    // return;
                }
                if (measurement.distance_mm < 250) {

                    intakeOne.set(ControlMode.PercentOutput, .0);
                    intakeTwo.set(ControlMode.PercentOutput, .25);
                    // transport.set(ControlMode.PercentOutput, 0.1);
                    // return;
                }

                var out = -laserFilter.calculate(notePID.calculate(measurement.distance_mm, 207));
                transport.set(TalonSRXControlMode.PercentOutput, MathUtil.clamp(out, -.5, .75));
                shooterSub.makeItGoBackwards();
                return;

            }
        } else if (isFeeding) {
            transport.set(ControlMode.PercentOutput, .7);
            intakeOne.set(ControlMode.PercentOutput, 0.5);
            intakeTwo.set(ControlMode.PercentOutput, 0.5);

        } else {

            intakeOne.set(ControlMode.PercentOutput, 0);
            intakeTwo.set(ControlMode.PercentOutput, 0);
            transport.set(ControlMode.PercentOutput, 0);

            shooterSub.turnOff();
        }
    }
}