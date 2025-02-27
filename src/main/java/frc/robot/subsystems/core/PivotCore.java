package frc.robot.subsystems.core;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.annotation.Nulls;

import frc.robot.Config;
import frc.robot.subsystems.EndoFactorSubsystem.EjectSpeed;
import frc.robot.subsystems.hardware.Motor3658;

public class PivotCore {
    //region Properies
    private final Motor3658 _motor = new Motor3658(Config.kPivotMotorId, Config.kCanbus)
        .setNeutralMode(NeutralModeValue.Brake)
        .setMaxSpeed(0.0625)
        .build();

    private final TalonFX m_PivotMotor = new TalonFX(Config.kPivotMotorId, Config.kCanbus); //Pivot Motor
    private final CANcoder s_PivotEncoder = new CANcoder(Config.kPivotEncoderId, Config.kCanbus);

    public enum PivotTarget {
        CoralIntake(-0.415),
        SafetyTarget(-0.366),
        Level1(-0.415),
        Level2AndLevel3(-0.366),
        Level4(-0.243),
        AlgaeIntake(0.128)
        ;
        double value;
        PivotTarget(double value) {
            this.value = value;
        }
    }
    
    private PivotTarget pt_PivotTarget;
    private EjectSpeed es_EjectSpeed = EjectSpeed.Level1;
    private final double d_PivotMotorSpeed = 0.0625;
    private boolean b_hasCoral;
    //endregion

    //region Methods
public void runPivotToPose(PivotTarget target) {
    double d_CurrentPose = getPivotPosition();

    double speed = Math.min(Math.max((target.value - d_CurrentPose) * 75, -1), 1);

    m_PivotMotor.set(speed * d_PivotMotorSpeed);
}

    public boolean isSafe() {
        return(getPivotPosition() >= PivotTarget.SafetyTarget.value - 0.004);
    }

    public boolean isFinished() {
        return(getPivotPosition() >= getPivotTarget().value - 0.004 && getPivotPosition() <= getPivotTarget().value + 0.004);
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
        return b_hasCoral;
    }

    public void setEjectSpeed(EjectSpeed es) {
        es_EjectSpeed = es;
    }
    //endregion
}
