package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    private TalonFX m_LiftMotor = new TalonFX(51, "3658CANivore");

    // Clamps on elevator connect to shallow cage
    // Clamps on chassis connect to deep cage
}
