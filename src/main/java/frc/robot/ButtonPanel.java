package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ButtonPanel {
    public enum Button {
        Stow(1, 1),
        ElevatorPosition2(2, 2),
        ElevatorPosition3(3, 3),
        ElevatorPosition4(4, 4),
        ElevatorPositionBarge(5, 5),
        ManualClaw1(6, 6),
        ManualClaw2(7, 7),
        ClawPositionCoral(12, 12),
        ClawPositionAlgae(13, 13),
        CoralIn(8, 8),
        CoralOut(9, 9),
        AlgaeIn(10, 10),
        AlgaeOut(11, 11),
        HONK(15),
        DeepClimbExtend(16),
        DeepClimbRetract(14);
        public final int value;
        public final int indicator;
        public boolean pressed;
        Button(int value) {
            this.value = value;
            this.indicator = 0;
            this.pressed = false;
        }
        Button(int value, int indicator) {
            this.value = value;
            this.indicator = indicator;
            this.pressed = false;
        }
    }

    private GenericHID button_panel;

    public ButtonPanel(int port) {
        button_panel = new GenericHID(port);
    }

    /**
     * Returns whether a button is held or not.
     * @param button The button to check
     * @return True or false
     */
    public boolean getButton(Button button) {
        return (button.pressed || button_panel.getRawButton(button.value));
    }

    /**
     * Returns whether a button has just been pressed.
     * @param button The button to check
     * @return True or false
     */
    public boolean getButtonPressed(Button button) {
        SmartDashboard.putBoolean("-"+button.name(), button.pressed);
        boolean p = button.pressed;
        button.pressed = false;
        return (p || button_panel.getRawButtonPressed(button.value));
    }

    public void setIndicatorLight(Button button, boolean on) {
        button_panel.setOutput(button.indicator, on);
    }

    public void AlgaeCancel() { // This command (should) cancel the algea button press at the end of autonomous in case that it is not cancelled during.
        Button.AlgaeIn.pressed = false;
        System.out.println("AlgaeCanceled");
    }
    
    public void AutonCancel() { // This command (should) cancel all button presses at the end of autonomous in case they are not calcelled during autonomous.
        for (Button panel : Button.values()) {
            panel.pressed = false;
        }
        System.out.println("ButtonsCanceled");
    }
}