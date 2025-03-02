package frc.robot.commands;

import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;

public class SwerveDriveCommand extends Command {
    private final SwerveDrivetrainSubsystem s_Swerve;
    private final ElevatorSubsystem s_Elevator;
    private final XboxController xb_Driver;

    private final double c_MaxSwerveSpeed = 5.21; // kSpeedAt12VoltsMps desired top speed
  	private final double c_MaxSwerveAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    public SwerveDriveCommand(SwerveDrivetrainSubsystem s, ElevatorSubsystem e, XboxController xb) {
        s_Swerve = s;
        s_Elevator = e; // Again, probably a bad practice, but we just need to read the encoder value.
        xb_Driver = xb;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forward = 0;
        double strafe = 0;
        double rotate = 0;
        double elevatorSpeedReduction = (s_Elevator.getEncoderValue()/Level.Coral4.value)*0.75; // From 0 to 1 (0% to 100%), how much do we reduce swerve speed?
        if (s_Swerve.getDriverOverride()) {
            forward = s_Swerve.getForward() * 0.125 * (1-elevatorSpeedReduction);
            strafe = s_Swerve.getStrafe() * 0.125 * (1-elevatorSpeedReduction);
            rotate = s_Swerve.getRotate() * (1-elevatorSpeedReduction);
        }
        else {
            forward = -xb_Driver.getLeftY() * (1-elevatorSpeedReduction);
            strafe = -xb_Driver.getLeftX() * (1-elevatorSpeedReduction);
            rotate = -xb_Driver.getRightX() * (1-elevatorSpeedReduction);
        }

        if (xb_Driver.getAButton()) {
            forward *= 0.25;
            strafe *= 0.25;
            rotate *= 0.25;
        }

        // if (xb_Driver.getLeftBumperButton() && xb_Driver.getRightBumperButton() && xb_Driver.getLeftTriggerAxis() > 0.5 && xb_Driver.getRightTriggerAxis() > 0.5 && xb_Driver.getAButton() && xb_Driver.getBButton() && xb_Driver.getXButton() && xb_Driver.getYButton() && xb_Driver.getLeftStickButton()) {
            // s_Swerve.Honk();
        // }
        // else {
        if (!s_Swerve.b_honking) {
            s_Swerve.setControl(drive.
                withVelocityX(forward * c_MaxSwerveSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(strafe * c_MaxSwerveSpeed) // Drive left with negative X (left)
                .withRotationalRate(rotate * c_MaxSwerveAngularRate) // Drive counterclockwise with negative X (left)
            );
        }

        SmartDashboard.putNumber("SWERVE - X", s_Swerve.getState().Pose.getX());
        SmartDashboard.putNumber("SWERVE - Y", s_Swerve.getState().Pose.getY());
        SmartDashboard.putNumber("SWERVE - Rotation", s_Swerve.getState().Pose.getRotation().getDegrees());

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}