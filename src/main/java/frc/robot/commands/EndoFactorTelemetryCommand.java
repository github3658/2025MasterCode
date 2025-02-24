package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonPanel;
import frc.robot.ButtonPanel.Button;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SameDayDeliverySubsystem;

public class EndoFactorTelemetryCommand extends Command {
    private SameDayDeliverySubsystem s_EndEffector;
    private ButtonPanel bp_Operator;
    private ElevatorSubsystem s_Elevator;

    public EndoFactorTelemetryCommand(SameDayDeliverySubsystem sdd, ButtonPanel bp, ElevatorSubsystem e) {
        s_EndEffector = sdd;
        bp_Operator = bp;
        s_Elevator = e;
        addRequirements(s_EndEffector);
    }

     @Override
    public void initialize() { }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Elevator Position", s_Elevator.getEncoderValue());
        SmartDashboard.putNumber("Pivot Position", s_EndEffector.getPivotPosition());
        SmartDashboard.putBoolean("Algae In", bp_Operator.getButton(Button.AlgaeIn));
        SmartDashboard.putBoolean("Algae Out", bp_Operator.getButton(Button.AlgaeOut));
        SmartDashboard.putBoolean("Claw Position Algae", bp_Operator.getButton(Button.ClawPositionAlgae));
        SmartDashboard.putBoolean("Claw Position Coral", bp_Operator.getButton(Button.ClawPositionCoral));
        SmartDashboard.putBoolean("Coral In", bp_Operator.getButton(Button.CoralIn));
        SmartDashboard.putBoolean("Coral Out", bp_Operator.getButton(Button.CoralOut));
        SmartDashboard.putBoolean("Deep Climb Extend", bp_Operator.getButton(Button.DeepClimbExtend));
        SmartDashboard.putBoolean("Deep Climb Retract", bp_Operator.getButton(Button.DeepClimbRetract));
        SmartDashboard.putBoolean("Elevator Position 2", bp_Operator.getButton(Button.ElevatorPosition2));
        SmartDashboard.putBoolean("Elevator Position 3", bp_Operator.getButton(Button.ElevatorPosition3));
        SmartDashboard.putBoolean("Elevator Position 4", bp_Operator.getButton(Button.ElevatorPosition4));
        SmartDashboard.putBoolean("Elevator Position Barge", bp_Operator.getButton(Button.ElevatorPositionBarge));
        SmartDashboard.putBoolean("Manual Claw 1", bp_Operator.getButton(Button.ManualClaw1));
        SmartDashboard.putBoolean("Manual Claw 2", bp_Operator.getButton(Button.ManualClaw2));
        SmartDashboard.putBoolean("Shallow Climb", bp_Operator.getButton(Button.ShallowClimb));
        SmartDashboard.putBoolean("Stow", bp_Operator.getButton(Button.Stow));
    }
}
