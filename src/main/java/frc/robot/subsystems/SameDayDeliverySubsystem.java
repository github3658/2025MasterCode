package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANrange;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class SameDayDeliverySubsystem extends SubsystemBase {
    public SameDayDeliverySubsystem() {
        pt_PivotTarget = PivotTarget.CoralIntake;
        setMotorConfig();
        zeroDeliveryPosition();
        zeroPivotPosition();
    }

    private final double d_MotorSpeed = 0.0625;
    
    private void setMotorConfig() {
        TalonFXConfiguration confPivot = new TalonFXConfiguration();
        confPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        TalonFXConfiguration confDelivery = new TalonFXConfiguration();
        confDelivery.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        confDelivery.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_DeliveryMotor.getConfigurator().apply(confDelivery);
        m_PivotMotor.getConfigurator().apply(confPivot);
    }

    //region Delivery
    //region Properies
    private final TalonFX m_DeliveryMotor = new TalonFX(Config.kDeliveryMotorId, Config.kCanbus); 
    private final CANrange s_DeliveryRange = new CANrange(Config.kDeliveryRangeId, Config.kCanbus);

    /** The max range for when we know coral is in the Endofactor. */ 
    private final double d_CoralDetectionRange = 10.0;
    private final double d_CoralIntakeTimerDuration = 5.0;
    /** The number of enoder ticks required to injest the coral from the distance sensor. */
    private final double d_CoralInTravel = 1000;
    /** The number of enoder ticks required to eject the coral from when the play presses the button. */
    private final double d_CoralOutTravel = 1000;

    private final Timer t_CoralIntakeTimer = new Timer();
    /** If true this prevents the player from ejecting the coral */
    private boolean b_IsCoralOutputDisabled; 
    /** If true this prevents the player from intaking the coral */
    private boolean b_IsCoralIntakeDisabled;
    private boolean b_IsCoralIntakeTimerEnabled;
    private boolean b_IsCoralRunningIntakeToPose;
    private boolean b_IsCoralEjecting;
    private double d_TargetCoralPosition;
    //endregion

    //region Methods
    //region Coral
    public SameDayDeliverySubsystem intakeCoral() {
        if (!b_IsCoralIntakeDisabled) {
            b_IsCoralIntakeDisabled = true; // Prevents the player's button press.
            b_IsCoralOutputDisabled = true; // Prevents the player's button press.
            m_DeliveryMotor.set(d_MotorSpeed);
            t_CoralIntakeTimer.reset();
            t_CoralIntakeTimer.start();
            b_IsCoralIntakeTimerEnabled = true; // Will start running the timer method in the command's execute method.
        }

        return this;
    }

    public SameDayDeliverySubsystem ejectCoral() {
        if (!b_IsCoralOutputDisabled) {
            b_IsCoralOutputDisabled = true;
            b_IsCoralEjecting = true; 
            m_DeliveryMotor.set(d_MotorSpeed);
            d_TargetCoralPosition = getDeliveryPosition() + d_CoralOutTravel;
        }

        return this;
    }

    public boolean getIsIntakeDisabled() { 
        return b_IsCoralIntakeDisabled;
    }

    public boolean getIsTimerEnabled() {
        return b_IsCoralIntakeTimerEnabled;
    }

    private void zeroDeliveryPosition() {
        m_DeliveryMotor.setPosition(0);
    }

    public double getDeliveryPosition() {
        return m_DeliveryMotor.getPosition().getValueAsDouble();
    }

    public double getDeliveryRange() {
        return s_DeliveryRange.getDistance().getValueAsDouble();
    }

    public SameDayDeliverySubsystem timerExecute() {
        if (b_IsCoralIntakeTimerEnabled) {
            if (getDeliveryRange() < d_CoralDetectionRange) { 
                stopTimer();
                d_TargetCoralPosition = getDeliveryPosition() + d_CoralInTravel;
                b_IsCoralRunningIntakeToPose = true; // Will start running the runIntakeToPose method in the command's execute method.
            }
            else if (t_CoralIntakeTimer.hasElapsed(d_CoralIntakeTimerDuration)) {
                stopTimer();
                m_DeliveryMotor.set(0);
                b_IsCoralIntakeDisabled = false;
                b_IsCoralOutputDisabled = false;
            }
        }
        return this;
    }

    private void stopTimer() {
        t_CoralIntakeTimer.stop();
        b_IsCoralIntakeTimerEnabled = false; 
    }

    public SameDayDeliverySubsystem runIntakeToPose() {
        if (b_IsCoralRunningIntakeToPose || b_IsCoralEjecting) {
            if (getPivotPosition() >= d_TargetCoralPosition) {
                m_DeliveryMotor.set(0);
                if (b_IsCoralRunningIntakeToPose) { 
                    b_IsCoralOutputDisabled = false; 
                    b_IsCoralRunningIntakeToPose = false;
                } else if (b_IsCoralEjecting) {
                    b_IsCoralIntakeDisabled = false;
                    b_IsCoralEjecting = false;
                }
            }
        }
        return this;
    }
    //endregion

    //region Algae
    public void intakeAlgae() { }

    public void ejectAlgae() {
    }
    //endregion
    //endregion
    //endregion

    //region Pivot
    //region Properies
    private final TalonFX m_PivotMotor = new TalonFX(Config.kPivotMotorId, Config.kCanbus); //Pivot Motor
    private final CANcoder s_PivotEncoder = new CANcoder(Config.kPivotEncoderId, Config.kCanbus);

    // TODO: Change to reflect the values of the CANcoder at these positions in the end.
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
    
    private PivotTarget pt_PivotTarget;
    //endregion

    //region Methods
    public boolean isSafe() {
        return(getPivotPosition() == PivotTarget.SafetyTarget.value);
    }

    public void setPivot(PivotTarget target) {
        pt_PivotTarget = target;
    }

    public void adjustCurrentPivot(double offset) {
        pt_PivotTarget.value += offset;
    }
    
    private void zeroPivotPosition() {
        m_PivotMotor.setPosition(0);
    }

    public double getPivotPosition() { 
        return m_PivotMotor.getPosition().getValueAsDouble();
    }

    public boolean isPivotTarget(PivotTarget target) {
        return pt_PivotTarget == target;
    }
    //endregion
    //endregion
}
