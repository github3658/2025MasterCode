package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SameDayDelivery extends SubsystemBase {
    // Entire subsystem pivots
    // Algae and coral get one motor each. intake and delivery
    // Camera will be mounted here for driver and robot vision

    private TalonFX m_pivot = new TalonFX(999, "3658CANivore");
    private TalonFX m_coral = new TalonFX(999, "3658CANivore");
    private TalonFX m_algae = new TalonFX(999, "3658CANivore");

    public SameDayDelivery() {
    }

    /**
     * Sets the delivery system pivot to the algae side or coral side, depending on the argument provided.
     * @param algae_side Whether to set the delivery pivot to the algae side.
     */
    public void SetPivot(boolean algae_side) {

    }

    /**
     * Eject the coral, or algae from the end effector.
     * @param algae_side
     */
    public void Eject(boolean algae_side) {

    }
}
