package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // Up and down, reads limit switches
    // 3 pre-programmed stages accessible by button panel
    // Adjusts swerve speed for safety

    private TalonFX m_LeftElevatorMotor = new TalonFX(41, "3658CANivore");
    private TalonFX m_RightElevatorMotor = new TalonFX(42, "3658CANivore");


    public enum Level {
        Level1(1),
        Level2(2),
        Level3(3),
        Level4(4),
        LevelBarge(5);
        public final int value;
        Level(int value) {
            this.value = value;
        }
    }

    private TalonFX m_Elevator1 = new TalonFX(999, "3658CANivore"); // Update motor ID
    private TalonFX m_Elevator2 = new TalonFX(999, "3658CANivore"); // Update motor ID

    private Level l_TargetLevel = Level.Level1;

    /**
     * Constructs the Elevator Subsystem
     */
    public ElevatorSubsystem() { }

    /**
     * Given an elevator level, will set the elevator subsystem to that level.
     * @param level The level to set.
     */
    public void setLevel(Level level) {
        l_TargetLevel = level;
    }

    /**
     * Returns the value of the elevator motor's encoder.
     * @return
     */
    public float getEncoderValue() {
        return 0;
    }

    /**
     * Returns the value of the elevator motor's encoder, adjusted to reflect the reef level (1-5).
     * @return
     */
    public float getEncoderLevel() {
        return 0;
    }

    /**
     * Returns the reef level that the elevator is trying to go to (1-5).
     * @return
     */
    public Level getTargetLevel() {
        return l_TargetLevel;
    }
}
