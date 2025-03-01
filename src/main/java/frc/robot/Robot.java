// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*; // This imports all of our subsystems.
import frc.robot.subsystems.ElevatorSubsystem.Level;
import frc.robot.subsystems.SameDayDeliverySubsystem.PivotTarget;
import frc.robot.ButtonPanel.Button;
import frc.robot.commands.*; // This imports all of our commands.
import frc.robot.commands.DriveToPoseCommand.Position;
import frc.robot.commands.autonomous.ButtonPanelPressCommand;
import frc.robot.commands.autonomous.WaitForTrue;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class Robot extends TimedRobot {
 
  // This is the driver's controller.
  // Typing "xb_Driver." will show you all of the XboxController methods, such as querying button inputs.
  private XboxController xb_Driver = new XboxController(0);

  // This is the operator's button panel.
  // This is a custom class I made that lets you check for input using a Button enum.
  private ButtonPanel bp_Operator = new ButtonPanel(1);

  private SwerveDrivetrainSubsystem s_Swerve = TunerConstants.createDrivetrain();
  private ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
  private SameDayDeliverySubsystem s_EndEffector = new SameDayDeliverySubsystem();
  private ClimbSubsystem s_ClimbSubsystem = new ClimbSubsystem();
  private LEDSubsystem s_LED = new LEDSubsystem();

  @Override
  public void robotInit() {
    s_Swerve.setDefaultCommand(new SwerveDriveCommand(s_Swerve, s_Elevator, xb_Driver));
    s_Elevator.setDefaultCommand(new ElevatorDefaultCommand(s_Elevator, bp_Operator, s_EndEffector));
    s_EndEffector.setDefaultCommand(new EndoFactorDefaultCommand(s_EndEffector, bp_Operator, s_Elevator));
    s_ClimbSubsystem.setDefaultCommand(new ClimbDefaultCommand(s_ClimbSubsystem, bp_Operator));
    s_Elevator.setLocked(true);
  }

  int test;
  boolean test2;

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Limelight Target X", LimelightHelpers.getTX(""));
    //SmartDashboard.putNumber("PivotEncoder2", s_EndEffector.getPivotPosition());
    //SmartDashboard.putBoolean("Stow Button", bp_Operator.getButton(Button.Stow));

    // If the elevator wants to move, take the End Effector out of unsafe positions.
    if (!s_Elevator.isFinished() && !s_EndEffector.isSafe()) {
      System.out.println("Move to safety position");
      s_EndEffector.setPivot(PivotTarget.SafetyTarget);
    }

    if (s_Elevator.isFinished() && s_Elevator.getElevatorLevel() == Level.Stow && (s_EndEffector.getPivotTarget() != PivotTarget.CoralIntake && s_EndEffector.getPivotTarget() != PivotTarget.AlgaeIntake)) {
      s_EndEffector.setPivot(PivotTarget.CoralIntake);
    }
    
    if (s_Elevator.getLocked() && s_EndEffector.isSafe() && !s_EndEffector.isPivotTarget(PivotTarget.CoralIntake)) {
      System.out.println("Unlock Elevator");
      s_Elevator.setLocked(false);
    }
    else if (!s_Elevator.getLocked() && !s_EndEffector.isSafe()) {
      System.out.println("Lock Elevator");
      s_Elevator.setLocked(true);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // We'll count the reef faces from 1 to 6

    new ParallelCommandGroup(
      new ParallelCommandGroup(
        new ElevatorDefaultCommand(s_Elevator, bp_Operator, s_EndEffector).ignoringDisable(true),
        new EndoFactorDefaultCommand(s_EndEffector, bp_Operator, s_Elevator).ignoringDisable(true)
      ),
      new SequentialCommandGroup(
        new DriveToPoseCommand(s_Swerve, Position.Face1LeftCoral.pose),
        new ButtonPanelPressCommand(Button.ElevatorPosition4, true),
        new WaitForTrue(() -> s_Elevator.isFinished()),
        new ButtonPanelPressCommand(Button.CoralOut, true),
        new WaitForTrue(() -> s_EndEffector.isFinished())
        //new ButtonPanelPressCommand(Button.CoralOut, true),
        //new ButtonPanelPressCommand(Button.Stow, true)
      )
    ).schedule();

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
		//   new DriveToPoseCommand(s_Swerve, new Pose2d(new Translation2d(1.5, 0), new Rotation2d())),
		//   new DriveToPoseCommand(s_Swerve, new Pose2d(new Translation2d(3.95, 1.5), new Rotation2d(Math.toRadians(-60)))),
		//   new DriveToPoseCommand(s_Swerve, new Pose2d(new Translation2d(3.95, 1.2), new Rotation2d(Math.toRadians(-60))))
		// ).schedule();
    //new Orangelight(s_Swerve).schedule();
    
    LimelightHelpers.setPipelineIndex(Config.kLimelight, 0);
  }

  @Override
  public void autonomousPeriodic() {
    // boolean hasTarget = LimelightHelpers.getTV(Config.kLimelight);

  //   if (hasTarget) {
  //     // Get the AprilTag ID
  //     double tagID = LimelightHelpers.getFiducialID(Config.kLimelight);

  //     // Get the translation of the detected AprilTag (in meters)
  //     double[] translation = LimelightHelpers.getBotPose(Config.kLimelight);

  //     // Convert the translation to inches (1 meter = 39.3701 inches)
  //     double translationXInches = translation[0] * 39.3701;
  //     double translationYInches = translation[1] * 39.3701;
  //     double translationZInches = translation[2] * 39.3701;

  //     // Display the detected information on the SmartDashboard
  //     SmartDashboard.putNumber("AprilTag ID", tagID);
  //     SmartDashboard.putString("AprilTag Status", "Target detected");
  //     SmartDashboard.putNumber("Translation X (inches)", translationXInches);
  //     SmartDashboard.putNumber("Translation Y (inches)", translationYInches);
  //     SmartDashboard.putNumber("Translation Z (inches)", translationZInches);
  // } else {
  //     SmartDashboard.putString("AprilTag Status", "No target detected");
  // }

    NetworkTable table = NetworkTableInstance.getDefault().getTable(Config.kLimelight);
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    double limelightMountAngleDegrees = 27.0;
    double limelightLenseHeightInches = 8.5;

    double goalHeightInches = 50.125;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLenseHeightInches) / Math.tan(angleToGoalRadians);

    SmartDashboard.putNumber("DISTANCE TO TAG", distanceFromLimelightToGoalInches);
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
  }
double counter = 0.0;
  @Override
  public void teleopPeriodic() {
    if (xb_Driver.getStartButtonPressed()) {
      s_Swerve.resetPose(new Pose2d());
    }
  }
  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
