package frc.robot.commands.autonomous;

import frc.robot.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForDelay extends Command {

    private double d_delay;

    public WaitForDelay(double seconds) {
        d_delay = seconds;
    }

    @Override
    public boolean isFinished() {
        return (d_delay -= 0.02) < 0;
    }
}
