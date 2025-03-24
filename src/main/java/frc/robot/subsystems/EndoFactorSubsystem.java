package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
                        .withProximityThreshold(0.07) //Changed to 0.1 for increased detection distance. Was 0.05 although that gave a possibility of detection distance dipping below the coral.
                        .withProximityHysteresis(0.01)
                )
                .withFovParams(
                    new FovParamsConfigs()
                        .withFOVRangeX(6.75)
                        .withFOVRangeY(6.75)
                )
        );
        PivotTarget.CoralIntake.value = s_PivotEncoder.getAbsolutePosition(true).getValueAsDouble();
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
    private final double d_PivotMotorSpeed = 0.135;
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

    private final double d_MaxAlgaePulseTime = 1.0;

    private final Timer t_CoralIntakeTimer = new Timer();
    private final Timer t_AlgaePulseTimer = new Timer();
    /** If true this prevents the player from ejecting the coral */
    private boolean b_IsCoralOutputDisabled; 
    /** If true this prevents the player from intaking the coral */
    private boolean b_IsCoralIntakeDisabled;
    private boolean b_IsCoralIntakeTimerEnabled;
    private boolean b_IsCoralRunningIntakeToPose;
    private boolean b_IsCoralEjecting;
    private double d_TargetCoralPosition;
    private boolean b_HoldingAlgae;

    //endregion

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
        double speed = (m_DeliveryMotor.getSupplyCurrent().getValueAsDouble() > c_DeliverySupplyCurrentLimit) ? 0 : -d_IntakeMotorSpeed;
        
        m_DeliveryMotor.set(speed);
    }

    public void ejectAlgae() {
        b_HoldingAlgae = false;
        t_AlgaePulseTimer.stop();
        t_AlgaePulseTimer.reset();
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

    public boolean hasAlgaeLimitSwitch() {
        SmartDashboard.putBoolean("Algae Limit Switch", dio_LimitSwitch.get());
        return dio_LimitSwitch.get();
    }

    public boolean hasAlgaeCurrentLimit() {
        return m_DeliveryMotor.getSupplyCurrent().getValueAsDouble() > c_DeliverySupplyCurrentLimit;
    }

    public boolean likelyHasAlgae() {
        return b_HoldingAlgae;
    }

    public boolean isAlgaePulseRunning() {
        return t_AlgaePulseTimer.isRunning();
    }

    private MusicTone mt_440 = new MusicTone(440);
    private MusicTone mt_0 = new MusicTone(0);
    public void beep(boolean beeping) {
        if (DriverStation.isTestEnabled() && beeping) {
            m_DeliveryMotor.setControl(mt_440);
        }
        else {
            m_DeliveryMotor.setControl(mt_0);
        }
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
        SafetyTarget(-0.36),
        Level1(-0.415),
        Level2AndLevel3(-0.36),
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
    
    m_PivotMotor.set(speed * d_PivotMotorSpeed);
}

    private boolean b_IsSafe;
    private int i_SafeSamples;
    private final int c_MaxSafeSamples = 15;
    public boolean isSafeSample() {
        return(getPivotPosition() >= PivotTarget.SafetyTarget.value - 0.01);
    }

    public boolean isSafe() {
        if (isSafeSample() != b_IsSafe) {
            i_SafeSamples ++;
            if (i_SafeSamples >= c_MaxSafeSamples) {
                b_IsSafe = isSafeSample();
            }
        }
        else {
            i_SafeSamples = 0;
        }
        return b_IsSafe;
    }

    public boolean isFinished() {
        return(getPivotPosition() >= getPivotTarget().value - d_EncoderLeewayRange && getPivotPosition() <= getPivotTarget().value + d_EncoderLeewayRange);
    }

    public void setPivot(PivotTarget target) {
        //System.out.println("Set pivot!");
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
        if (s_DeliveryRange.getIsDetected().getValue()) {
            SmartDashboard.putNumber("Detected Distance", s_DeliveryRange.getDistance().getValueAsDouble());
            SmartDashboard.putNumber("Coral_In_Strength", s_DeliveryRange.getSignalStrength().getValueAsDouble());
        }
        SmartDashboard.putNumber("Coral Signal Strength", s_DeliveryRange.getSignalStrength().getValueAsDouble());
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
        SmartDashboard.putNumber("Endofactor - measure CORAL DISTANCE", s_DeliveryRange.getDistance().getValueAsDouble());
        //SmartDashboard.putNumber("Endofactor - Delivery stator current", m_DeliveryMotor.getStatorCurrent().getValueAsDouble());
        //SmartDashboard.putNumber("Endofactor - Delivery voltage", m_DeliveryMotor.getMotorVoltage().getValueAsDouble());
        if (maxDeliveyCurrent < m_DeliveryMotor.getSupplyCurrent().getValueAsDouble()) 
            maxDeliveyCurrent= m_DeliveryMotor.getSupplyCurrent().getValueAsDouble();

        if (maxPivotCurrent < m_PivotMotor.getSupplyCurrent().getValueAsDouble())
            maxPivotCurrent = m_PivotMotor.getSupplyCurrent().getValueAsDouble();

        SmartDashboard.putNumber("MaxDeliveryCurrent", maxDeliveyCurrent);
        SmartDashboard.putNumber("MaxPivotCurrent", maxPivotCurrent);

        if (getPivotTarget() == PivotTarget.AlgaeIntake && (hasAlgaeLimitSwitch() || hasAlgaeCurrentLimit())) { // When we first obtain algae, set this boolean to true.
            b_HoldingAlgae = true;
        }

        if (!(hasAlgaeLimitSwitch() || hasAlgaeCurrentLimit()) && b_HoldingAlgae && !t_AlgaePulseTimer.isRunning()) { // If the limit switch is unset, and we should be holding algae, start the pulse timer
            t_AlgaePulseTimer.start();
        }

        if (b_HoldingAlgae && t_AlgaePulseTimer.isRunning() && t_AlgaePulseTimer.get() < d_MaxAlgaePulseTime) { // Run the intake while the timer is less than the max
            intakeAlgae();
        }
        else if ((hasAlgaeLimitSwitch() || hasAlgaeCurrentLimit()) || t_AlgaePulseTimer.get() >= d_MaxAlgaePulseTime || getPivotTarget() != PivotTarget.AlgaeIntake) { // Cancel the intake if we get the algae again or time runs out.
            if (t_AlgaePulseTimer.get() >= d_MaxAlgaePulseTime || getPivotTarget() != PivotTarget.AlgaeIntake) {
                b_HoldingAlgae = false;
            }
            t_AlgaePulseTimer.stop();
            t_AlgaePulseTimer.reset();
        }
        
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
