package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final double d_ClimbSpeed = 0.0625;

    private TalonFX m_LiftMotor = new TalonFX(51, "3658CANivore");

    public ClimbSubsystem() {
        m_LiftMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        
    }

    public void extend() {
        m_LiftMotor.set(d_ClimbSpeed);
    }

    public void retract() {
        m_LiftMotor.set(-d_ClimbSpeed);
    }

    public void stop() {
        m_LiftMotor.set(0);
    }
}
