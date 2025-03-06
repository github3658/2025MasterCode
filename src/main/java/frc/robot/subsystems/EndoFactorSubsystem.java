package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.core.Delivery.DeliveryCore;
import frc.robot.subsystems.core.Pivot.PivotCore;
import frc.robot.subsystems.core.Pivot.PivotTargets;

public class EndoFactorSubsystem extends SubsystemBase {

    private final PivotCore _pivotCore = new PivotCore();
    private final DeliveryCore _deliveryCore = new DeliveryCore();

    public EndoFactorSubsystem() {
        _pivotCore.setPivot(PivotTargets.CoralIntake)
            .getMotor().zeroPosition();
    }
   
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Endofactor - current position", _pivotCore.getMotor().getCurrentPosition());
        SmartDashboard.putNumber("Endofactor - target position", _pivotCore.getCurrentTarget().value);
        SmartDashboard.putNumber("Endofactor - dt range", _deliveryCore.getCoral().getRange().getDistance());
        SmartDashboard.putNumber("Endofactor - samples", _deliveryCore.getCoral().getRange().getActualCount());
        SmartDashboard.putBoolean("Endofactor - is safe?", _pivotCore.isSafe());
        SmartDashboard.putBoolean("Endofactor - has coral?", _deliveryCore.getCoral().hasCoral());
    }
}
