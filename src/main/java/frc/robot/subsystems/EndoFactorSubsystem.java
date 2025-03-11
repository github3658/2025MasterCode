package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.subsystems.LEDSubsystem.Color;

public class EndoFactorSubsystem extends SubsystemBase {

    private DigitalInput dio_LimitSwitch = new DigitalInput(Config.kAlgeaLimitSwitch);
    public EndoFactorSubsystem() {
        pt_PivotTarget = PivotTarget.CoralIntake;
        setMotorConfig();
        zeroDeliveryPosition();
        zeroPivotPosition();
        s_DeliveryRange.getConfigurator().apply(
            new CANrangeConfiguration()
                .withProximityParams(
                    new ProximityParamsConfigs()
                        .withProximityThreshold(0.05)
                        .withProximityHysteresis(0.01)
                )
                .withFovParams(
                    new FovParamsConfigs()
                        .withFOVRangeX(6.75)
                        .withFOVRangeY(6.75)
                )
        );
    }

    private final double d_IntakeMotorSpeed = 0.125;
    public enum EjectSpeed {
        Level1(0.3),
        Level2(0.3),
        Level3(0.3),
        Level4(0.3);
        double value;
        private EjectSpeed(double value) {
            this.value = value;
        }
    }
    private EjectSpeed es_EjectSpeed = EjectSpeed.Level1;
    private final double d_PivotMotorSpeed = 0.145;
    private final double c_PivotSupplyCurrentLimit = 3;
    private boolean b_hasCoral;
    
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

    private final double d_EncoderLeewayRange = 0.005;
    private final double c_DeliverySupplyCurrentLimit = 5.0;

    /** The max range for when we know coral is in the Endofactor. */
    private final double d_CoralIntakeTimerDuration = 5.0;
    /** The number of encoder ticks required to injest the coral from the distance sensor. */
    private final double d_CoralInTravel = 3.0;
    /** The number of encoder ticks required to eject the coral from when the play presses the button. */
    private final double d_CoralOutTravel = 7.5;

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
    public EndoFactorSubsystem intakeCoral() {
        if (!b_IsCoralIntakeDisabled) {
            b_IsCoralIntakeDisabled = true; // Prevents the player's button press.
            b_IsCoralOutputDisabled = true; // Prevents the player's button press.
            m_DeliveryMotor.set(d_IntakeMotorSpeed);
            t_CoralIntakeTimer.reset();
            t_CoralIntakeTimer.start();
            b_IsCoralIntakeTimerEnabled = true; // Will start running the timer method in the command's execute method.
        }
        return this;
    }

    public EndoFactorSubsystem ejectCoral() {
        if (!b_IsCoralOutputDisabled || true) {
            b_IsCoralOutputDisabled = true;
            b_IsCoralEjecting = true; 
            m_DeliveryMotor.set(es_EjectSpeed.value);
            d_TargetCoralPosition = getDeliveryPosition() + d_CoralOutTravel;
        }

        return this;
    }

    public boolean getIsIntakeDisabled() { 
        return b_IsCoralIntakeDisabled;
    }

    public boolean getIsOutputDisabled() { 
        return b_IsCoralOutputDisabled;
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

    public EndoFactorSubsystem timerExecute() {
        if (b_IsCoralIntakeTimerEnabled) {
            if (hasCoral()) {
                System.out.println("DETECT CORAL");
                stopTimer();
                d_TargetCoralPosition = getDeliveryPosition() + d_CoralInTravel;
                b_IsCoralRunningIntakeToPose = true; // Will start running the runIntakeToPose method in the command's execute method.
            }
            else if (t_CoralIntakeTimer.hasElapsed(d_CoralIntakeTimerDuration)) {
                System.out.println("TIMEOUT");
                stopTimer();
                m_DeliveryMotor.set(0);
                b_IsCoralIntakeDisabled = false;
                b_IsCoralOutputDisabled = false;
            }
        }
        return this;
    }

    private void stopTimer() {
        System.out.println("STOP TIMER");
        t_CoralIntakeTimer.stop();
        b_IsCoralIntakeTimerEnabled = false; 
    }

    public EndoFactorSubsystem runIntakeToPose() {
        SmartDashboard.putNumber("INTAKE - POSITION", getDeliveryPosition());
        SmartDashboard.putNumber("INTAKE - TARGET", d_TargetCoralPosition);
        if (b_IsCoralRunningIntakeToPose || b_IsCoralEjecting) {
            if (getDeliveryPosition() >= d_TargetCoralPosition) {
                System.out.println("REACHED TARGET");
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
    public void intakeAlgae() {
        //TODO: Should this be changed to listen to the limit switch, hasAlgae.
        double speed = m_DeliveryMotor.getSupplyCurrent().getValueAsDouble() > c_DeliverySupplyCurrentLimit ? 0 : -d_IntakeMotorSpeed;
        
        m_DeliveryMotor.set(speed);
    }

    public void ejectAlgae() {
        m_DeliveryMotor.set(0.50);
    }

    public void stopAlgaeDelivery() {
        m_DeliveryMotor.set(0);
    }

    public boolean canGoToCoralPos() {
        return b_canGoToCoralPos;
    }

    boolean b_canGoToCoralPos = true;
    public void canGoToCoralPos(boolean allowed) {
        b_canGoToCoralPos = allowed;
    }

    public boolean hasAlgae() {
        SmartDashboard.putBoolean("Algae Limit Switch", dio_LimitSwitch.get());
        return dio_LimitSwitch.get();
        // return (m_DeliveryMotor.getSupplyCurrent().getValueAsDouble() > c_DeliverySupplyCurrentLimit);
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
        CoralIntake(-0.415),
        SafetyTarget(-0.366),
        Level1(-0.415),
        Level2AndLevel3(-0.366),
        Level4(-0.243),
        AlgaeIntake(0.128),
        AlgaeIntakeStow(0.050),
        ;
        double value;
        PivotTarget(double value) {
            this.value = value;
        }
    }
    
    private PivotTarget pt_PivotTarget;
    //endregion

    private double _maxCurrent = 0;
    //region Methods
public void runPivotToPose(PivotTarget target) {
    double d_CurrentPose = getPivotPosition();
    double c_Current = m_PivotMotor.getSupplyCurrent().getValueAsDouble();
    double speed = c_Current > c_PivotSupplyCurrentLimit ? 0 : Math.min(Math.max((target.value - d_CurrentPose) * 35, -1), 1);
    
if (c_Current > _maxCurrent) _maxCurrent = c_Current;
    SmartDashboard.putNumber("Algae Current", _maxCurrent);
    SmartDashboard.putNumber("Algae Speed", speed* d_PivotMotorSpeed);
    m_PivotMotor.set(speed * d_PivotMotorSpeed);
}

    public boolean isSafe() {
        return(getPivotPosition() >= PivotTarget.SafetyTarget.value - 0.004);
    }

    public boolean isFinished() {
        return(getPivotPosition() >= getPivotTarget().value - d_EncoderLeewayRange && getPivotPosition() <= getPivotTarget().value + d_EncoderLeewayRange);
    }

    public void setPivot(PivotTarget target) {
        System.out.println("Set pivot!");
        pt_PivotTarget = target;
    }

    public void adjustCurrentPivot(double offset) {
        pt_PivotTarget.value += offset;
        pt_PivotTarget.value = Math.max(pt_PivotTarget.value, -0.415);
    }
    
    private void zeroPivotPosition() {
        m_PivotMotor.setPosition(0);
    }

    public double getPivotPosition() { 
        return s_PivotEncoder.getAbsolutePosition(true).getValueAsDouble();
    }

    public boolean isPivotTarget(PivotTarget target) {
        return pt_PivotTarget == target;
    }

    public PivotTarget getPivotTarget() {
        return pt_PivotTarget;
    }

    public boolean hasCoral() {
        return s_DeliveryRange.getIsDetected().getValue();
    }

    public void setEjectSpeed(EjectSpeed es) {
        es_EjectSpeed = es;
    }
    //endregion
    //endregion

    private double maxDeliveyCurrent = 0;
    private double maxPivotCurrent = 0;

    @Override
    public void periodic() {
        SmartDashboard.putString("Endofactor - target position", pt_PivotTarget.name());
        SmartDashboard.putBoolean("Endofactor - is safe?", isSafe());
        SmartDashboard.putBoolean("Endofactor - has coral?", hasCoral());
        SmartDashboard.putNumber("Endofactor - measure", s_DeliveryRange.getDistance().getValueAsDouble());
        //SmartDashboard.putNumber("Endofactor - Delivery stator current", m_DeliveryMotor.getStatorCurrent().getValueAsDouble());
        //SmartDashboard.putNumber("Endofactor - Delivery voltage", m_DeliveryMotor.getMotorVoltage().getValueAsDouble());
        if (maxDeliveyCurrent < m_DeliveryMotor.getSupplyCurrent().getValueAsDouble()) 
            maxDeliveyCurrent= m_DeliveryMotor.getSupplyCurrent().getValueAsDouble();

        if (maxPivotCurrent < m_PivotMotor.getSupplyCurrent().getValueAsDouble())
            maxPivotCurrent = m_PivotMotor.getSupplyCurrent().getValueAsDouble();

        SmartDashboard.putNumber("MaxDeliveryCurrent", maxDeliveyCurrent);
        SmartDashboard.putNumber("MaxPivotCurrent", maxPivotCurrent);
        
        //System.out.println("PivotSupply"+m_PivotMotor.getSupplyCurrent().getValueAsDouble());
        //System.out.println("DeliverySupply"+m_DeliveryMotor.getSupplyCurrent().getValueAsDouble());

        //runPivotToPose(pt_PivotTarget);
        b_hasCoral = hasCoral();

        // Disable coral controls in algae mode for Operator clarity
        if (getPivotTarget() == PivotTarget.AlgaeIntake || getPivotTarget() == PivotTarget.AlgaeIntakeStow) {
            b_hasCoral = false;
        }

        b_IsCoralOutputDisabled = !b_hasCoral;
        b_IsCoralIntakeDisabled = b_hasCoral;

        if (b_IsCoralRunningIntakeToPose) {
            b_IsCoralOutputDisabled = true;
            b_IsCoralOutputDisabled = true;
        }
    }
}
