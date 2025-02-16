package frc.robot.commands;

import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveDrive extends Command {
    private final CommandSwerveDrivetrain s_Swerve;
    private final XboxController xb_Driver;

    private double d_forward;
    private double d_strafe;
    private double d_rotate;

    private boolean b_override;

    private final double c_MaxSwerveSpeed = 5.21; // kSpeedAt12VoltsMps desired top speed
  	private final double c_MaxSwerveAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    public SwerveDrive(CommandSwerveDrivetrain s, XboxController xb) {
        s_Swerve = s;
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
        if (s_Swerve.getDriverOverride()) {
            forward = s_Swerve.getForward() * 0.125;
            strafe = s_Swerve.getStrafe() * 0.125;
            rotate = s_Swerve.getRotate();
        }
        else {
            forward = -xb_Driver.getLeftY()*0.25; //Used to be 0.25
            strafe = -xb_Driver.getLeftX()*0.25;
            rotate = -xb_Driver.getRightX();
        }

        if (xb_Driver.getAButton()) {
            forward *= 0.25;
            strafe *= 0.25;
            rotate *= 0.25;
        }

        s_Swerve.setControl(drive.
            withVelocityX(forward * c_MaxSwerveSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(strafe * c_MaxSwerveSpeed) // Drive left with negative X (left)
            .withRotationalRate(rotate * c_MaxSwerveAngularRate) // Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}