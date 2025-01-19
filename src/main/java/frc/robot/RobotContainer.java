// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

//import com.ctre.phoenix.mechanisms.swerve.SwerveModule;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.command.DefaultClimb;
import frc.robot.command.DefaultIntake;
import frc.robot.command.DefaultShooter;
import frc.robot.command.DefaultSwerve;
import frc.robot.command.ResetClimb;
import frc.robot.command.autolime.AutoAlignTags;
import frc.robot.command.autolime.AutoRotate;
import frc.robot.command.autolime.NoteRotationAlign;
import frc.robot.command.autolime.autoSequences.CenterAuto;
import frc.robot.command.autolime.autoSequences.LeftAuto;
import frc.robot.command.autolime.autoSequences.RightAuto;
import frc.robot.command.autolime.autoSequences.ThreeNoteCenterAuto;
import frc.robot.subsytems.Arms;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import frc.robot.subsytems.SwerveSubsystem;

public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  // controllers
  private final Joystick primaryJoy = new Joystick(0);
  private final XboxController secondaryController = new XboxController(1);

  // TODO subsystems
  private final SwerveSubsystem swerveSub = new SwerveSubsystem();
  private final Arms Arms = new Arms();
  private final Shooter shooter = new Shooter();
  private final Intake mouth = new Intake(shooter);

  // commands
  private final DefaultSwerve defaultSwerve = new DefaultSwerve(primaryJoy, swerveSub);
  private final DefaultIntake intakeTransport = new DefaultIntake(mouth, secondaryController, shooter, primaryJoy);
  private final DefaultClimb climbCommand = new DefaultClimb(primaryJoy, Arms);
  private final DefaultShooter shootCommand = new DefaultShooter(primaryJoy, secondaryController, shooter, mouth);

  public RobotContainer() throws FileVersionException, IOException, ParseException {
    swerveSub.setDefaultCommand(defaultSwerve);
    shooter.setDefaultCommand(shootCommand);
    mouth.setDefaultCommand(intakeTransport);
    Arms.setDefaultCommand(climbCommand);
    configureBindings();

    autoChooser.addOption("PathPlanner Line", AutoBuilder.buildAuto("ForwardBack"));
    autoChooser.addOption("PathPlanner Square", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Square")));
    autoChooser.addOption("right", new RightAuto(swerveSub, shooter, mouth));
    autoChooser.addOption("center", new CenterAuto(swerveSub, shooter, mouth));
    autoChooser.addOption("left", new LeftAuto(swerveSub, shooter, mouth));
    autoChooser.addOption("3_Note_Center", new ThreeNoteCenterAuto(swerveSub, shooter, mouth));
    autoChooser.addOption("rotate", new AutoRotate(swerveSub, 30, 0.25));
    // autoChooser.addOption("right",new ThreeAutoToRuleThemAll(swerveSub, shooter, mouth));
    Shuffleboard.getTab("auto").add(autoChooser);
    Shuffleboard.getTab("Drive").add("ResetClimb", new ResetClimb(Arms));

    // Shooter shooterSub = new Shooter();
    // AutoDrive step = new AutoDrive(swerveSub, 0, 0); // TODO
    // push commands to pathweaver auto
    // NamedCommands.registerCommand("drive", step);

    // autoChooser = AutoBuilder.buildAutoChooser();

    // Shuffleboard.getTab("autoChooser").add(autoChooser);

    new Thread(() -> {
      LimelightHelpers.getLatestResults("limelight-back");
    }).start();

    swerveSub.resetOmetry(new Pose2d(1, 1, new Rotation2d()));
  }

  private void configureBindings() {
    new JoystickButton(primaryJoy, 3).whileTrue(new AutoAlignTags(swerveSub));
    // new JoystickButton(primaryJoy, 8).whileTrue(new PathPlannerAuto("New New
    // new JoystickButton(primaryJoy, 11).whileTrue(new PathPlannerAuto("RIGHTAUTO"));
    new JoystickButton(primaryJoy, 8).whileTrue(Commands.run(() -> {swerveSub.resetOmetry(new Pose2d(1,1, new Rotation2d()));}));
    new JoystickButton(primaryJoy, 9).whileTrue(Commands.run(() -> {System.out.println(swerveSub.getPose());}));
    new JoystickButton(primaryJoy, 10).whileTrue(new NoteRotationAlign(swerveSub));
    new JoystickButton(primaryJoy, 11).onTrue(Commands.runOnce(swerveSub::botposewithapriltag, swerveSub));
   // new JoystickButton(primaryJoy, 11).whileTrue(new AutoDriveAndTrackNote(swerveSub, 2.5, 0.3));
    // Auto"));
  }

  public Command getAutonomousCommand() {
    var command = autoChooser.getSelected();


    // Command command = null;
    if (command != null) {
      return command;
    } else {
      return new InstantCommand();
    }
    // return new OneAutoToRuleThemAll(swerveSub, shooter, mouth);
  }

  // public void resetFieldOrientation() {
  //   // swerveSub.zeroYaw();
  // }
}