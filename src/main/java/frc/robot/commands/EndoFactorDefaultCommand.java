package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonPanel;
import frc.robot.ButtonPanel.Button;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SameDayDeliverySubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;
import frc.robot.subsystems.SameDayDeliverySubsystem.EjectSpeed;
import frc.robot.subsystems.SameDayDeliverySubsystem.PivotTarget;

public class EndoFactorDefaultCommand extends Command {

    private SameDayDeliverySubsystem s_EndEffector;
    private ButtonPanel bp_Operator;
    private ElevatorSubsystem s_Elevator; 
    // Ideally, we'd only include this subsystem in commands that necessitate reading information from it.
    // Since we're only reading information from it, there should be no conflicts.

    private PivotTarget pt_IdealTarget; // Depending on the elevator height, what position should we eject coral?
    private boolean b_LightFlash;
    private int i_LightTimer;

    public EndoFactorDefaultCommand(SameDayDeliverySubsystem sdd, ButtonPanel bp, ElevatorSubsystem e) {
        s_EndEffector = sdd;
        bp_Operator = bp;
        s_Elevator = e;
        addRequirements(s_EndEffector);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        // Depending on elevator position, set the coral eject position.
        pt_IdealTarget = PivotTarget.CoralIntake;
        if (s_Elevator.getTargetLevel() == Level.Stow) {
            pt_IdealTarget = PivotTarget.CoralIntake;
            s_EndEffector.setEjectSpeed(EjectSpeed.Level1);
        }
        else if (s_Elevator.getTargetLevel() == Level.Coral2 || s_Elevator.getTargetLevel() == Level.Coral3) {
            pt_IdealTarget = PivotTarget.SafetyTarget;
            if (s_Elevator.getTargetLevel() == Level.Coral2) {
                s_EndEffector.setEjectSpeed(EjectSpeed.Level2);
            }
            else {
                s_EndEffector.setEjectSpeed(EjectSpeed.Level3);
            }
        }
        else if (s_Elevator.getTargetLevel() == Level.Coral4) {
            pt_IdealTarget = PivotTarget.Level4;
            s_EndEffector.setEjectSpeed(EjectSpeed.Level4);
        }

        // Pivot Control
        if (bp_Operator.getButtonPressed(Button.ClawPositionAlgae)) {
            System.out.println("AlgaePose");
            s_EndEffector.setPivot(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake);
            s_EndEffector.canGoToCoralPos(false);
        }
        else if (bp_Operator.getButtonPressed(Button.ClawPositionCoral)) {
            System.out.println("CoralPose");
            if (s_EndEffector.canGoToCoralPos()) {
                s_EndEffector.setPivot(SameDayDeliverySubsystem.PivotTarget.CoralIntake);
            }
        }

        // Pivot adjustment
        if (bp_Operator.getButton(Button.ManualClaw1)) {
            s_EndEffector.adjustCurrentPivot(-0.01);
        }
        else if (bp_Operator.getButton(Button.ManualClaw2)) {
            s_EndEffector.adjustCurrentPivot(0.01);
        }

        // Intake
        if (bp_Operator.getButton(Button.AlgaeIn)) {
            System.out.println("AlgaeIn");
            if (s_EndEffector.isPivotTarget(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake)) {
                s_EndEffector.intakeAlgae(); // TODO
            }
            else {
                s_EndEffector.setPivot(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake);
            }
        }
        else if (bp_Operator.getButtonPressed(Button.CoralIn) && !s_EndEffector.getIsIntakeDisabled() && s_Elevator.getElevatorLevel() == Level.Stow) {
            System.out.println("Coral In");
            //TODO: Refactor to if the pivot is positioned then we position it first and the run the intake coral.
            s_EndEffector.intakeCoral();
        }

        s_EndEffector.timerExecute().runIntakeToPose();
        s_EndEffector.runPivotToPose(s_EndEffector.getPivotTarget());

        // Eject
        if (bp_Operator.getButton(Button.AlgaeOut)) {
            System.out.println("Algae Out");
            if (s_EndEffector.isPivotTarget(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake)) {
                s_EndEffector.ejectAlgae(); // TODO
                s_EndEffector.canGoToCoralPos(true);
            }
            else {
                s_EndEffector.setPivot(SameDayDeliverySubsystem.PivotTarget.AlgaeIntake);
            }
        }
        else if (bp_Operator.getButtonPressed(Button.CoralOut)) {
            System.out.println("Coral Out");
            if (s_EndEffector.isPivotTarget(pt_IdealTarget)) {
                s_EndEffector.ejectCoral();
            }
            else {
                s_EndEffector.setPivot(pt_IdealTarget);
            }
        }

        if (bp_Operator.getButton(Button.AlgaeOut) == false && bp_Operator.getButton(Button.AlgaeIn) == false && s_EndEffector.isPivotTarget(PivotTarget.AlgaeIntake)) {
            s_EndEffector.stopAlgaeDelivery();
        }

        bp_Operator.setIndicatorLight(Button.CoralIn, !s_EndEffector.getIsIntakeDisabled());
        bp_Operator.setIndicatorLight(Button.AlgaeIn, s_EndEffector.getPivotTarget() == PivotTarget.AlgaeIntake);
        bp_Operator.setIndicatorLight(Button.AlgaeOut, s_EndEffector.getPivotTarget() == PivotTarget.AlgaeIntake);
        bp_Operator.setIndicatorLight(Button.CoralOut, !s_EndEffector.getIsOutputDisabled());
        bp_Operator.setIndicatorLight(Button.ClawPositionAlgae, true);
        bp_Operator.setIndicatorLight(Button.ClawPositionCoral, s_EndEffector.canGoToCoralPos());
        if (s_EndEffector.getPivotTarget() == pt_IdealTarget && s_EndEffector.isFinished() && s_EndEffector.hasCoral()) {
            bp_Operator.setIndicatorLight(Button.CoralOut, b_LightFlash);
        }

        i_LightTimer++;
        if (i_LightTimer > 10) {
            i_LightTimer = 0;
            b_LightFlash = !b_LightFlash;
        }
    }
}
