package frc.robot.commands.autonomous;

import frc.robot.Logger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class LogLocationCommand extends InstantCommand {
    private SwerveDrivetrainSubsystem s_Swerve;
    private String s_Title;
    public LogLocationCommand(SwerveDrivetrainSubsystem swerve, String title) {
        s_Swerve = swerve;
        s_Title = title;
    }

    @Override
    public void initialize() {
        System.out.println("-----"+s_Title+"-----");
        System.out.println("X " + s_Swerve.getState().Pose.getX());
        System.out.println("Y " + s_Swerve.getState().Pose.getY());
        System.out.println("ROT " + s_Swerve.getState().Pose.getRotation().getDegrees());
    }
}
