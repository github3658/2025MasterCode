package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonPanel;
import frc.robot.ButtonPanel.Button;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;

public class ElevatorDefaultCommand extends Command {

    private ElevatorSubsystem s_Elevator;
    private ButtonPanel bp_Operator;

    public ElevatorDefaultCommand(ElevatorSubsystem e, ButtonPanel bp) {
        s_Elevator = e;
        bp_Operator = bp;
        addRequirements(s_Elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Setting the elevator level waits for the end effector to be in a safe position to do so,
        // so it's okay to use the setLevel function like this.
        if (bp_Operator.getButtonPressed(Button.Stow)) {
            s_Elevator.setLevel(Level.Level1);
        }
        else if (bp_Operator.getButtonPressed(Button.ElevatorPosition2)) {
            s_Elevator.setLevel(Level.Level2);
        }
        else if (bp_Operator.getButtonPressed(Button.ElevatorPosition3)) {
            s_Elevator.setLevel(Level.Level3);
        }
        else if (bp_Operator.getButtonPressed(Button.ElevatorPosition4)) {
            s_Elevator.setLevel(Level.Level4);
        }
        else if (bp_Operator.getButtonPressed(Button.ElevatorPositionBarge)) {
            s_Elevator.setLevel(Level.LevelBarge);
        }
    }
}
