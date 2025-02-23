package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANrange;

import java.lang.reflect.Type;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SameDayDeliverySubsystem extends SubsystemBase {
    // These values are temporary, and should reflect the values of the CANcoder at these positions in the end.
    public enum PivotTarget {
        CoralIntake(0),
        SafetyTarget(10),
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
    private CANrange sensor_CANrange = new CANrange(61, "3658CANivore");
    private CANcoder sensor_CANcoder = new CANcoder(33, "3658CANivore");
    private boolean b_IntakeDisabled;
    private final Timer t_IntakeTimer = new Timer();
    private boolean t_IntakeTimerEnabled;
    private final double d_IntakeTimerDuration = 5; //TODO: Adjust timer duration.
    private double d_TargetDeliveryPosition;
    private boolean b_runningIntakeToPose;

    private final double defaultMotorSpeed = 0.0625; //Default motor speed.

    private boolean b_OutputDisabled; 
    private boolean b_Ejecting;

    private PivotTarget pt_PivotTarget;


    public SameDayDeliverySubsystem() {
        pt_PivotTarget = PivotTarget.CoralIntake;
        setMotorConfig();
        zeroDeliveryPosition();
        zeroPivotPosition();
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
        if (b_IntakeDisabled == false) {
            b_IntakeDisabled = true;
            b_OutputDisabled = true;
            m_EndoDeivery.set(defaultMotorSpeed);
            t_IntakeTimer.reset();
            t_IntakeTimer.start();
            t_IntakeTimerEnabled = true;
        }
    }

    public boolean isB_IntakeDisabled() { 
        return b_IntakeDisabled;
    }

    public boolean isTimerEnabled() {
        return t_IntakeTimerEnabled;
        }

    public SameDayDeliverySubsystem timerExecute() {
        if (t_IntakeTimerEnabled) {
            if (sensor_CANrange.getDistance().getValueAsDouble() < 10) { //TODO: set distance instead of 10
                stopTimer();
                d_TargetDeliveryPosition = m_EndoDeivery.getPosition().getValueAsDouble() + 1000; //TODO: 1000 needs to be tuned to an actual value of encoder.
                b_runningIntakeToPose = true;
            }
            else if (t_IntakeTimer.hasElapsed(d_IntakeTimerDuration)) {
                stopTimer();
                m_EndoDeivery.set(0);
                b_IntakeDisabled = false;
                b_OutputDisabled = false;
            }
        }
        return this;
    }
    


    private void stopTimer() {
        t_IntakeTimer.stop();
        t_IntakeTimerEnabled = false; 
    }

    public SameDayDeliverySubsystem runIntakeToPose() {
        if (b_runningIntakeToPose || b_Ejecting) {
            if (getPivotPosition() >= d_TargetDeliveryPosition) {
                m_EndoDeivery.set(0);
                b_runningIntakeToPose = false; //TODO: Make sure to re-enable the intake when the coral is ejected.
                b_OutputDisabled = false; 
            }
        }
        return this;
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
        if (!b_OutputDisabled) {
            b_OutputDisabled = true;
            b_Ejecting = true; 
            m_EndoDeivery.set(defaultMotorSpeed);
            d_TargetDeliveryPosition = getDeliveryPosition() + 1000; //TODO: Get value for output
        }
    }

    /**
     * Adjusts the value of the current PivotTarget manually. This offset persists until the robot is rebooted.
     * @param offset
     */
    public void adjustCurrentPivot(double offset) {
        pt_PivotTarget.value += offset;
    }

    private double getPivotPosition() { 
        return m_EndoPivot.getPosition().getValueAsDouble();
    }

    private double getDeliveryPosition() {
        return m_EndoDeivery.getPosition().getValueAsDouble();
    }

    public boolean isSafe() {
        return(getPivotPosition() == PivotTarget.SafetyTarget.value);
    }
    
    private void setMotorConfig() {
        TalonFXConfiguration confPivot = new TalonFXConfiguration();
        confPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        TalonFXConfiguration confDelivery = new TalonFXConfiguration();
        confDelivery.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        confDelivery.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


        m_EndoDeivery.getConfigurator().apply(confDelivery);
        m_EndoPivot.getConfigurator().apply(confPivot);
    }

    private void zeroDeliveryPosition() {
        m_EndoDeivery.setPosition(0);
    }
    
    private void zeroPivotPosition() {
        m_EndoPivot.setPosition(0);
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
