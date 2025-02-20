package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonPanel;
import frc.robot.ButtonPanel.Button;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SameDayDeliverySubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;
import frc.robot.subsystems.SameDayDeliverySubsystem.PivotTarget;

public class EndoFactorDefaultCommand extends Command {

    private SameDayDeliverySubsystem s_EndEffector;
    private ButtonPanel bp_Operator;
    private ElevatorSubsystem s_Elevator; 
    // Ideally, we'd only include this subsystem in commands that necessitate reading information from it.
    // Since we're only reading information from it, there should be no conflicts.

    private PivotTarget pt_IdealTarget; // Depending on the elevator height, what position should we eject coral?

    public EndoFactorDefaultCommand(SameDayDeliverySubsystem sdd, ButtonPanel bp, ElevatorSubsystem e) {
        s_EndEffector = sdd;
        bp_Operator = bp;
        s_Elevator = e;
        addRequirements(s_EndEffector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //TODO: If the elevate is orders to move up from it's store position we need to ensure the Endofactor is up to it's Safety Position.
        // Depending on elevator position, set the coral eject position.
        pt_IdealTarget = PivotTarget.CoralIntake;
        if (s_Elevator.getTargetLevel() == Level.Level1) {
            pt_IdealTarget = PivotTarget.Level1;
        }
        else if (s_Elevator.getTargetLevel() == Level.Level2 || s_Elevator.getTargetLevel() == Level.Level3) {
            pt_IdealTarget = PivotTarget.Level2AndLevel3;
        }
        else if (s_Elevator.getTargetLevel() == Level.Level4) {
            pt_IdealTarget = PivotTarget.Level4;
        }

        // Pivot Control
        if (bp_Operator.getButtonPressed(Button.ClawPositionAlgae)) {
            s_EndEffector.setPivot(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake);
        }
        else if (bp_Operator.getButtonPressed(Button.ClawPositionCoral)) {
            s_EndEffector.setPivot(SameDayDeliverySubsystem.PivotTarget.CoralIntake);
        }

        // Pivot adjustment
        if (bp_Operator.getButton(Button.ManualClaw1)) {
            s_EndEffector.adjustCurrentPivot(-0.01);
        }
        else if (bp_Operator.getButton(Button.ManualClaw2)) {
            s_EndEffector.adjustCurrentPivot(0.01);
        }

        // Intake
        if (bp_Operator.getButtonPressed(Button.AlgaeIn)) {
            if (s_EndEffector.isPivotTarget(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake)) {
                s_EndEffector.intakeAlgae();
            }
            else {
                s_EndEffector.setPivot(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake);
            }
        }
        else if (bp_Operator.getButtonPressed(Button.CoralIn)) {
            //TODO: Prevent this from working if the distance sensor is detecting coral.
            if (s_EndEffector.isPivotTarget(SameDayDeliverySubsystem.PivotTarget.CoralIntake)) {
                s_EndEffector.intakeCoral();
            }
            else {
                s_EndEffector.setPivot(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake);
            }
        }

        // Eject
        if (bp_Operator.getButtonPressed(Button.AlgaeOut)) {
            if (s_EndEffector.isPivotTarget(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake)) {
                s_EndEffector.ejectAlgae();
            }
            else {
                s_EndEffector.setPivot(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake);
            }
        }
        else if (bp_Operator.getButtonPressed(Button.CoralOut)) {
            if (s_EndEffector.isPivotTarget(pt_IdealTarget)) {
                s_EndEffector.ejectCoral();
            }
            else {
                s_EndEffector.setPivot(pt_IdealTarget);
            }
        }
    }
}
