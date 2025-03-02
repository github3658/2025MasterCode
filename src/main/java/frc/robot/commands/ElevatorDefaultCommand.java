package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonPanel;
import frc.robot.ButtonPanel.Button;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndoFactorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;
import frc.robot.subsystems.EndoFactorSubsystem.PivotTarget;

public class ElevatorDefaultCommand extends Command {

    private ElevatorSubsystem s_Elevator;
    private EndoFactorSubsystem s_EndEffector;
    private ButtonPanel bp_Operator;

    public ElevatorDefaultCommand(ElevatorSubsystem e, ButtonPanel bp, EndoFactorSubsystem s) {
        s_Elevator = e;
        bp_Operator = bp;
        s_EndEffector = s;
        addRequirements(s_Elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Level currentLevel = s_Elevator.getElevatorLevel();
        bp_Operator.setIndicatorLight(Button.Stow, currentLevel == Level.Stow || currentLevel == Level.LevelBarge);
        bp_Operator.setIndicatorLight(Button.ElevatorPosition2, currentLevel == Level.Coral2);
        bp_Operator.setIndicatorLight(Button.ElevatorPosition3, currentLevel == Level.Coral3 || currentLevel == Level.Algea1);
        bp_Operator.setIndicatorLight(Button.ElevatorPosition4, currentLevel == Level.Coral4 || currentLevel == Level.Algea2);
        // Barge level isn't set properly, ignore it.

        // Setting the elevator level waits for the end effector to be in a safe position to do so,
        // so it's okay to use the setLevel function like this.
        if (bp_Operator.getButtonPressed(Button.Stow)) {
            s_Elevator.setLevel(Level.Stow);
        }
        else if (bp_Operator.getButtonPressed(Button.ElevatorPosition2)) {
            s_Elevator.setLevel(Level.Coral2);
        }
        else if (bp_Operator.getButtonPressed(Button.ElevatorPosition3)) {
            s_Elevator.setLevel(Level.Coral3);
        }
        else if (bp_Operator.getButtonPressed(Button.ElevatorPosition4)) {
            s_Elevator.setLevel(Level.Coral4);
        }
        else if (bp_Operator.getButtonPressed(Button.ElevatorPositionBarge)) {
            s_Elevator.setLevel(Level.LevelBarge);
        }

        // Modified positions for algae.
        if (s_EndEffector.getPivotTarget() == PivotTarget.AlgaeIntake) {
            if (s_Elevator.getTargetLevel() == Level.Coral3) {
                s_Elevator.setLevel(Level.Algea1);
            }
            else if (s_Elevator.getTargetLevel() == Level.Coral4) {
                s_Elevator.setLevel(Level.Algea2);
            }
        }
    }
}
