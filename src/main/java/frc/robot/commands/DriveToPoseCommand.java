package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class DriveToPoseCommand extends Command {

    // 1 UNIT ~ 50 cm

    // These are the positions the robot should drive to.
    public static final Pose2d Origin = new Pose2d(new Translation2d(0.0,0.0), Rotation2d.fromDegrees(0.0));

    // The maximum Swerve speed
    private final double c_MaxSwerveSpeed = 5.12; // kSpeedAt12VoltsMps desired top speed
  	
    // The maximum Swerve rotation speed
    private final double c_MaxSwerveAngularRate = 3.0 * Math.PI; // 3/4 of a rotation per second max angular velocity
    
    // The Swerve acceleration time
    private final double c_AccelTime = 25.0;

    // If our speed is less than this value, we should stop.
    private final double c_SwerveRampDeadzone = 0.1;

    // This is a multiplier for the Swerve speed, so we accelerate and slow down before switching targets.
    private double d_SwerveRamp;

    // The number of "frames" that have passed. 
    // We count how long the robot has been still to see if we should end the command.
    private int i_Frames;

    // This is the pose that the robot wants to go to.
    private Pose2d p_TargetPose;

    private double d_Forward;
    private double d_Strafe;
    private double d_Rotate;

    private final SwerveRequest.FieldCentric drive_field = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private SwerveDrivetrainSubsystem s_Swerve;

    public DriveToPoseCommand(SwerveDrivetrainSubsystem s, Pose2d target) {
        s_Swerve = s;
        p_TargetPose = target;//new Pose2d(new Translation2d(target.getX()/50, target.getY()/50), new Rotation2d(0));
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

        SmartDashboard.putNumber("AUTON ROT TARGET",p_TargetPose.getRotation().getRadians());
        SmartDashboard.putNumber("AUTON ROT CURRENT",currentPose.getRotation().getRadians());

        if (Math.abs(d_Forward) > c_SwerveRampDeadzone || Math.abs(d_Strafe) > c_SwerveRampDeadzone || Math.abs(d_Rotate) > c_SwerveRampDeadzone) {
            d_SwerveRamp = Math.min(d_SwerveRamp+1/c_AccelTime,1);
        }
        else {
            d_SwerveRamp = Math.max(d_SwerveRamp-1/c_AccelTime,0);
        }

        s_Swerve.setControl(drive_field.
            withVelocityX(d_Forward * d_SwerveRamp * c_MaxSwerveSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(d_Strafe * d_SwerveRamp * c_MaxSwerveSpeed) // Drive left with negative X (left)
            .withRotationalRate(d_Rotate * d_SwerveRamp * c_MaxSwerveAngularRate)
        );
    }

    @Override
    public boolean isFinished() {
        i_Frames++;
        return (Math.abs(d_Forward) < c_SwerveRampDeadzone && Math.abs(d_Strafe) < c_SwerveRampDeadzone && Math.abs(d_Rotate) < c_SwerveRampDeadzone && i_Frames > 5);
    }

    // This is a bad function name.
    // It converts a distance to a reasonable forward/strafe speed.
    private double toReasonableValue(double dist) {
        return Math.min(Math.max(dist/4,-0.5),0.5);
    }
}
