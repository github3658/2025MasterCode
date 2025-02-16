// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is the main robot file.
// We will create instances of our subsystems here and get input from the driver and operator.
// This is also where we will have our autonomous programs stored.

package frc.robot;

import frc.robot.subsystems.*; // This imports all of our subsystems.
import frc.robot.ButtonPanel.Buttons;
import frc.robot.commands.*; // This imports all of our commands.
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.GenericHID;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
 
  // This is the driver's controller.
  // Typing "xb_driver." will show you all of the XboxController methods, such as querying button inputs.
  private XboxController xb_driver = new XboxController(0);

  //private ButtonPanel bp_operator = new ButtonPanel(1);

  private CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  private Elevator s_Elevator = new Elevator();

  private SwerveDrive com_AwesomeDrive = new SwerveDrive(s_Swerve, xb_driver);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    s_Swerve.setDefaultCommand(com_AwesomeDrive);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Limelight Target X", LimelightHelpers.getTX(""));
    //SmartDashboard.putBoolean("Stow Button", bp_operator.GetButton(Buttons.Stow));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // We'll count the reef faces from 1 to 6, clockwise from the face toward the barge

    // REEF FACE 1
    // new SequentialCommandGroup(
		//   new DriveToPose(s_Swerve, new Pose2d(new Translation2d(3, 0), new Rotation2d()))
		// ).schedule();

    // FACE 6
    // X = 4
    // Y = -1.6
    // ROT = 58

    // FACE 5
    // X = 6
    // Y = -1.4
    // ROT = 135

    // FACE 4
    // X = 6.3
    // Y = 0
    // ROT = 180

    // FACE 3
    // X = 5.5
    // Y = 1.35
    // ROT = -118.6

    // FACE 2
    // X = 3.95
    // Y = 1.2
    // ROT = -60
    // new SequentialCommandGroup(
		//   new DriveToPose(s_Swerve, new Pose2d(new Translation2d(1.5, 0), new Rotation2d())),
		//   new DriveToPose(s_Swerve, new Pose2d(new Translation2d(3.95, 1.5), new Rotation2d(Math.toRadians(-60)))),
		//   new DriveToPose(s_Swerve, new Pose2d(new Translation2d(3.95, 1.2), new Rotation2d(Math.toRadians(-60))))
		// ).schedule();
    //new Orangelight(s_Swerve).schedule();
  }

  @Override
  public void autonomousPeriodic() {}
    
  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (xb_driver.getStartButtonPressed()) {
      s_Swerve.resetPose(new Pose2d());
    }
  }
  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
  @Override
  public void testExit() {}
}
