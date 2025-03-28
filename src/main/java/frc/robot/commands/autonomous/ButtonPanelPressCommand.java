package frc.robot.commands.autonomous;

import frc.robot.Logger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ButtonPanel.Button;

public class ButtonPanelPressCommand extends InstantCommand {

    private Button bt_Button;
    private boolean b_pressed;

    public ButtonPanelPressCommand(Button button, boolean pressed) {
        bt_Button = button;
        b_pressed = pressed;
    }

    @Override
    public void initialize() {
        bt_Button.pressed = b_pressed;
    }
}
