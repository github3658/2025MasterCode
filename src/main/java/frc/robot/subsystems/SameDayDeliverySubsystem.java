package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SameDayDeliverySubsystem extends SubsystemBase {
    // These values are temporary, and should reflect the values of the CANcoder at these positions in the end.
    public enum PivotTarget {
        CoralIntake(0),
        Level1(50),
        Level2AndLevel3(100),
        Level4(200),
        AlgaeIntake(300)
        ;
        double value;
        PivotTarget(double value) {
            this.value = value;
        }
    }

    private TalonFX m_EndoPivot = new TalonFX(31, "3658CANivore"); //Pivot Motor
    private TalonFX m_EndoDeivery = new TalonFX(32, "3658CANivore"); //Both Endofactors Input & Output
    private CANrange sensor_CANrange = new CANrange(999, "3658CANivore");
    private CANcoder sensor_CANcoder = new CANcoder(999, "3658CANivore");

    private PivotTarget pt_PivotTarget;

    public SameDayDeliverySubsystem() {
        pt_PivotTarget = PivotTarget.CoralIntake;
    }

    /**
     * Sets the delivery system pivot to the PivotTarget provided.
     * @param target Which PivotTarget to go to
     */
    public void setPivot(PivotTarget target) {
        pt_PivotTarget = target;
    }

    /**
     * Intake algae.
     */
    public void intakeAlgae() {

    }

    /**
     * Intake coral.
     */
    public void intakeCoral() {
        //TODO: We will need to disable the player button and take over with a timer once the distance sensor detects coral.
    }

     /**
     * Eject algae.
     */
    public void ejectAlgae() {

    }

    /**
     * Eject coral.
     */
    public void ejectCoral() {

    }

    /**
     * Adjusts the value of the current PivotTarget manually. This offset persists until the robot is rebooted.
     * @param offset
     */
    public void adjustCurrentPivot(double offset) {
        pt_PivotTarget.value += offset;
    }

    /**
     * Returns whether the current pivot target is the one provided.
     * @param target A PivotTarget value to check
     * @return Whether the provided value is our current target.
     */
    public boolean isPivotTarget(PivotTarget target) {
        return pt_PivotTarget == target;
    }
}
