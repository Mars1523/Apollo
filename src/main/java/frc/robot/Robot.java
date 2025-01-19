// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "dont choose btw";
  // private static final String kCustomAuto2 = "dont choose btw 2";
  // private String m_autoselected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private RobotContainer robotContainer;
  // private DigitalInput dio = new DigitalInput(1);

  @Override
  public void robotInit() {
    try {
      robotContainer = new RobotContainer();
    } catch (FileVersionException | IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    // this.autonomousCommand = robotContainer.getAutonomousCommand();

    // m_chooser.setDefaultOption("def", kDefaultAuto);
    // m_chooser.addOption("dntchoose", kCustomAuto);
    // m_chooser.addOption("2", kCustomAuto2);
    // SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {

    // if (dio.get()) {
    // robotContainer.resetFieldOrientation();
    // }

  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    // if (autonomousCommand != null) {
    // if (autonomousCommand.isFinished() == false) {
    // switch (m_autoselected) {
    // case kCustomAuto:
    // ((OneAutoToRuleThemAll) autonomousCommand).firstAuto();
    // autonomousCommand.schedule();
    // break;
    // case kCustomAuto2:
    // ((OneAutoToRuleThemAll) autonomousCommand).firstAuto();
    // autonomousCommand.schedule();
    // break;
    // case kDefaultAuto:
    // default:
    // ((OneAutoToRuleThemAll) autonomousCommand).firstAuto();
    // break;
    // }
    // }
    // }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    if(autonomousCommand != null){
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
