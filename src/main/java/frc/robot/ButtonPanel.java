package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonPanel {
    public enum Button {
        Stow(1, 1),
        ElevatorPosition2(2, 2),
        ElevatorPosition3(3, 3),
        ElevatorPosition4(4, 4),
        ElevatorPositionBarge(5, 5),
        ManualClaw1(6),
        ManualClaw2(7),
        ClawPositionCoral(8, 6),
        ClawPositionAlgae(9, 7),
        CoralIn(10, 8),
        CoralOut(11, 9),
        AlgaeIn(12, 10),
        AlgaeOut(13, 11),
        ShallowClimb(14),
        DeepClimbExtend(15),
        DeepClimbRetract(16);
        public final int value;
        public final int indicator;
        Button(int value) {
            this.value = value;
            this.indicator = 0;
        }
        Button(int value, int indicator) {
            this.value = value;
            this.indicator = indicator;
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
        return button_panel.getRawButton(button.value);
    }

    /**
     * Returns whether a button has just been pressed.
     * @param button The button to check
     * @return True or false
     */
    public boolean getButtonPressed(Button button) {
        return button_panel.getRawButtonPressed(button.value);
    }
}