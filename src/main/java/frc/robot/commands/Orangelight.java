package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import java.lang.System;
import java.lang.Math;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Orangelight extends Command { //This should probably be in commands. Also probably an absolute mess.
    double TX = LimelightHelpers.getTX("");
    double TY = LimelightHelpers.getTY("");
    double[] NotTag = LimelightHelpers.getT2DArray("");
    double Tag = NotTag[9];
    boolean Has_Target = LimelightHelpers.getTV("");
    double Angle = LimelightHelpers.getTA("");

    private CommandSwerveDrivetrain s_Swerve;

    public Orangelight(CommandSwerveDrivetrain s) {
        s_Swerve = s;
        if (Has_Target) {
            System.out.print("X=" + TX + " Y=" + TY);
            System.out.print("TAGID =" + Tag);
        }
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        s_Swerve.setDriverOverride(true);
    }

    @Override
    public void execute() {
        AlignX();
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.setDriverOverride(false);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(s_Swerve.getStrafe()) < 0.01);
    }

    public void AlignX() {
        s_Swerve.setStrafe(Math.min(Math.max((TX/4), -0.25), 0.25));
    }
}