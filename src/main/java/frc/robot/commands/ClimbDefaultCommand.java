package frc.robot.commands;

import frc.robot.ButtonPanel;
import frc.robot.ButtonPanel.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonPanel;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDefaultCommand extends Command {
    private ClimbSubsystem s_Climb;
    private ButtonPanel bp_Operator;

    public ClimbDefaultCommand(ClimbSubsystem climb, ButtonPanel bp) {
        s_Climb = climb;
        bp_Operator = bp;
        addRequirements(s_Climb);
    }

    @Override
    public void execute() {
        if (bp_Operator.getButton(Button.DeepClimbExtend)) {
            s_Climb.extend();
        }
        else if (bp_Operator.getButton(Button.DeepClimbRetract)) {
            s_Climb.retract();
        }
        else {
            s_Climb.stop();
        }
    }
}
