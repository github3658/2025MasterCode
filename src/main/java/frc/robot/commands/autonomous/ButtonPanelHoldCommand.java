package frc.robot.commands.autonomous;

import frc.robot.Logger;

import frc.robot.ButtonPanel.Button;
import edu.wpi.first.wpilibj2.command.Command;

public class ButtonPanelHoldCommand extends Command {
 public ButtonPanelHoldCommand(Button button, Boolean held) {
    boolean b_ButtonHeld = held;
    Button bt_Button = button;
 }
    @Override
    public void initialize() {}

    @Override 
    public void execute() {}

    @Override 
    public boolean isFinished() {
        return true;
    }
}
