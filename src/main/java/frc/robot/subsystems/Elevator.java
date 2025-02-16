package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    // Up and down, reads limit switches
    // 3 pre-programmed stages accessible by button panel
    // Adjusts swerve speed for safety

    public enum Levels {
        Level1(1),
        Level2(2),
        Level3(3),
        Level4(4),
        LevelBarge(5);
        public final int value;
        Levels(int value) {
            this.value = value;
        }
    }

    private TalonFX m_elevator = new TalonFX(999, "3658CANivore"); // Update motor ID

    /**
     * Constructs the Elevator Subsystem
     */
    public Elevator() {
    }

    /**
     * Given an elevator level, will set the elevator subsystem to that level.
     * @param level The level to set.
     */
    public void SetLevel(Levels level) {

    }

    /**
     * Returns the value of the elevator motor's encoder.
     * @return
     */
    private float GetEncoderValue() {
        return 0;
    }

    /**
     * Returns the value of the elevator motor's encoder, adjusted to reflect the reef level (1-4).
     * @return
     */
    private float GetEncoderLevel() {
        return 0;
    }
}
