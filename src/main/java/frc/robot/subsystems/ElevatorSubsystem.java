package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        public final double value;
        Level(double value) {
            this.value = value;
        }
    }

    private final double c_AcceptableEncoderRange = 10;

    private TalonFX m_Elevator1 = new TalonFX(41, "3658CANivore");
    private TalonFX m_Elevator2 = new TalonFX(42, "3658CANivore");

    private boolean b_locked;
    private Level l_TargetLevel = Level.Level1;

    /**
     * Constructs the Elevator Subsystem
     */
    public ElevatorSubsystem() {
        m_Elevator1.setPosition(0);
        m_Elevator2.setPosition(0);
        m_Elevator1.setNeutralMode(NeutralModeValue.Brake);
        m_Elevator2.setNeutralMode(NeutralModeValue.Brake);
        m_Elevator2.setControl(new Follower(41, true));
    }

    @Override
    public void periodic() {
        if (!b_locked && !isFinished()) {
            double speed = (getTargetLevel().value - getEncoderValue()) / 50; // Set speed to encoder difference
            speed = Math.min(Math.max(speed, -0.125), 0.125); // Clamp speed
            m_Elevator1.set(speed);
        }
        else {
            m_Elevator1.set(0);
        }
    }

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
    public double getEncoderValue() {
        return m_Elevator1.getPosition().getValueAsDouble();
    }

    /**
     * Returns the closest reef level (1-Barge).
     * @return
     */
    public Level getElevatorLevel() {
        double v = getEncoderValue();
        double distance = 999999;
        Level closest = Level.Level1;
        for (Level l : Level.values()) {
            if (Math.abs(l.value - v) < distance) {
                distance = Math.abs(l.value - v);
                closest = l;
            }
        }
        return closest;
    }

    /**
     * Returns the reef level that the elevator is trying to go to (1-5).
     * @return
     */
    public Level getTargetLevel() {
        return l_TargetLevel;
    }

    /**
     * Return whether the elevator has met its target level
     * @return
     */
    public boolean isFinished() {
        double v = getEncoderValue();
        return (v > l_TargetLevel.value - c_AcceptableEncoderRange) && (v < l_TargetLevel.value + c_AcceptableEncoderRange);
    }

    /**
     * Disables the elevator from moving or enables the elevator to move.
     * @param lock
     */
    public void setLocked(boolean lock) {
        b_locked = lock;
    }

    /**
     * Returns whether the elevator is locked.
     * @return
     */
    public boolean getLocked() {
        return b_locked;
    }
}
