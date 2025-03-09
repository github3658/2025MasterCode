package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.Level;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class DriveToPoseCommand extends Command {
    // These are the positions the robot should drive to.
    // The origin is the center of the field, behind the robot starting line.
    public enum Position {
        // Can be used from any starting position
        Origin(0, 0, 0),
        Pullout(0.5, 0, 0),
        PulloutTest(2,0, 0),
        PulloutTurnTest(0, 0, 90),

        // CENTER ONLY
        Reef10ACoral(2.875, 0, 0), 
        Reef10ABackup(2, 0, 0),
        Reef10AlgaeBackup(2, -0.4, 0),
        Reef10Algae(3.9, -0.4, 0, 0.35),
        FaceProcessor(2.85, 6.3, 90),
        FleeFromProcessor(5.0, 6.0, 90),
        
        // RIGHT ONLY
        Reef9ACoral(4.544, -5.189, -58.9),
        CoralStationBackup(11.861, -2.722, -125.75),
        CoralStation(12.90, -1.70, -125.75),
        Reef8ACoral(7.1, -6.025, -119.9),
        Reef8ABackup(8.1, -5.025, -119.9),


        // LEFT ONLY
        Reef11ACoral(4.544, 5.189, 58.9),
        invCoralStationBackup(11.861, 2.722, 125.75),
        invCoralStation(12.90, 1.70, 125.75),
        Reef12ACoral(7.1, 6.025, 119.9),
        Reef12ABackup(8.1, 5.025, 119.9),
// TODO: Check in practice field for alignment.

        ;
        public double x, y, angle, maxspeed;
        public Pose2d pose;
        Position(double x, double y, double angle) {
            this.x = x;
            this.y = y;
            this.angle = angle;
            this.maxspeed = 1;
            this.pose = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(angle));
        }
        Position(double x, double y, double angle, double maxspeed) {
            this.x = x;
            this.y = y;
            this.angle = angle;
            this.maxspeed = maxspeed;
            this.pose = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(angle));
        }
    }

    // 1 UNIT ~ 50 cm

    // The maximum Swerve speed
    private final double c_MaxSwerveSpeed = 5.12; // kSpeedAt12VoltsMps desired top speed
  	
    // The maximum Swerve rotation speed
    private final double c_MaxSwerveAngularRate = 3.0 * Math.PI; // 3/4 of a rotation per second max angular velocity
    
    // The Swerve acceleration time
    private final double c_AccelTime = 25.0;

    // If our speed is less than this value, we should stop.
    private double c_SwerveRampDeadzone = 0.1;

    // This is a multiplier for the Swerve speed, so we accelerate and slow down before switching targets.
    private double d_SwerveRamp;

    // The number of "frames" that have passed. 
    // We count how long the robot has been still to see if we should end the command.
    private int i_Frames;

    // This is the pose that the robot wants to go to.
    private Position p_Target;
    private Pose2d p_TargetPose;

    private double d_Forward;
    private double d_Strafe;
    private double d_Rotate;

    private final SwerveRequest.FieldCentric drive_field = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private SwerveDrivetrainSubsystem s_Swerve;
    private ElevatorSubsystem s_Elevator;

    public DriveToPoseCommand(SwerveDrivetrainSubsystem s, ElevatorSubsystem e, Position target) {
        s_Swerve = s;
        s_Elevator = e;
        p_Target = target;
        p_TargetPose = target.pose;//new Pose2d(new Translation2d(target.getX()/50, target.getY()/50), new Rotation2d(0));
        addRequirements(s_Swerve);
    }

    public DriveToPoseCommand(SwerveDrivetrainSubsystem s, ElevatorSubsystem e, Position target, double deadzone) {
        s_Swerve = s;
        s_Elevator = e;
        p_Target = target;
        p_TargetPose = target.pose;//new Pose2d(new Translation2d(target.getX()/50, target.getY()/50), new Rotation2d(0));
        c_SwerveRampDeadzone = deadzone;
        addRequirements(s_Swerve);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d currentPose = s_Swerve.getState().Pose;

        d_Forward = toReasonableValue(p_TargetPose.getX() - currentPose.getX());
        d_Strafe = toReasonableValue(p_TargetPose.getY() - currentPose.getY());
        d_Rotate = toReasonableValue(p_TargetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

        if (Math.abs(d_Forward) > c_SwerveRampDeadzone || Math.abs(d_Strafe) > c_SwerveRampDeadzone || Math.abs(d_Rotate) > c_SwerveRampDeadzone) {
            d_SwerveRamp = Math.min(d_SwerveRamp+1/c_AccelTime,1);
        }
        else {
            d_SwerveRamp = Math.max(d_SwerveRamp-1/c_AccelTime,0);
        }

        double elevatorSpeedReduction = (s_Elevator.getEncoderValue()/Level.Coral4.value)*0.35; // From 0 to 1 (0% to 100%), how much do we reduce swerve speed?
        
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            s_Swerve.setControl(drive_field.
                withVelocityX(-d_Forward * d_SwerveRamp * (1-elevatorSpeedReduction) *  c_MaxSwerveSpeed * p_Target.maxspeed) // Drive forward with negative Y (forward)
                .withVelocityY(-d_Strafe * d_SwerveRamp * (1-elevatorSpeedReduction) * c_MaxSwerveSpeed * p_Target.maxspeed) // Drive left with negative X (left)
                .withRotationalRate(d_Rotate * d_SwerveRamp * c_MaxSwerveAngularRate)
            );
        } else {
            s_Swerve.setControl(drive_field.
                withVelocityX(d_Forward * d_SwerveRamp * (1-elevatorSpeedReduction) *  c_MaxSwerveSpeed * p_Target.maxspeed) // Drive forward with negative Y (forward)
                .withVelocityY(d_Strafe * d_SwerveRamp * (1-elevatorSpeedReduction) * c_MaxSwerveSpeed * p_Target.maxspeed) // Drive left with negative X (left)
                .withRotationalRate(d_Rotate * d_SwerveRamp * c_MaxSwerveAngularRate)
            );
        }

        System.out.println("Can Coder: " + s_Swerve.getModule(0).getEncoder().getPosition().getValueAsDouble());
        System.out.println("X POS: " + s_Swerve.getState().Pose.getX());
        System.out.println("Y POS: " + s_Swerve.getState().Pose.getY());
        System.out.println("ROT: " + s_Swerve.getState().Pose.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        i_Frames++;
        return (Math.abs(d_Forward) < c_SwerveRampDeadzone && Math.abs(d_Strafe) < c_SwerveRampDeadzone && Math.abs(d_Rotate) < c_SwerveRampDeadzone && i_Frames > 5);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.setControl(drive_field.
            withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
        System.out.println("Reached " + p_Target.name());
        System.out.println("X " + s_Swerve.getState().Pose.getX());
        System.out.println("Y " + s_Swerve.getState().Pose.getY());
        System.out.println("ROT " + s_Swerve.getState().Pose.getRotation().getDegrees());
        //System.out.println("AT " + DriverStation.getMatchTime());
    }

    // This is a bad function name.
    // It converts a distance to a reasonable forward/strafe speed.
    private double toReasonableValue(double dist) {
        return Math.min(Math.max(dist/2,-0.65),0.65);
    }
}
