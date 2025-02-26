package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class ElevatorSubsystem extends SubsystemBase {
    // Up and down, reads limit switches
    // 3 pre-programmed stages accessible by button panel
    // Adjusts swerve speed for safety

    private TalonFX m_LeftElevatorMotor = new TalonFX(41, Config.kCanbus);
    private TalonFX m_RightElevatorMotor = new TalonFX(42, Config.kCanbus);


    public enum Level {
        Stow(0),
        Coral1(0),
        Coral2(12),
        Coral3(50),
        Coral4(124),
        Algea1(54),
        Algea2(87),
        LevelBarge(8);
        public final double value;
        Level(double value) {
            this.value = value;
        }
    }

    private final double c_AcceptableEncoderRange = 10;

    private TalonFX m_LeftElevator = new TalonFX(Config.kLeftElevatorMotor, Config.kCanbus);
    private TalonFX m_RightElevator = new TalonFX(Config.kRightElevatorMotor, Config.kCanbus);

    private boolean b_locked;
    private Level l_TargetLevel = Level.Stow;

    /**
     * Constructs the Elevator Subsystem
     */
    public ElevatorSubsystem() {
        m_LeftElevator.setPosition(0);
        m_RightElevator.setPosition(0);
        setElevatorConfig();
        //m_RightElevator.setControl(new Follower(Config.kLeftElevatorMotor, true)); TODO: Uncomment this when done with getting encoder positions.
    }

    private void setElevatorConfig() {
        TalonFXConfiguration cElevatorMotorConfig = new TalonFXConfiguration();
        cElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_LeftElevatorMotor.getConfigurator().apply(cElevatorMotorConfig);
        m_RightElevatorMotor.getConfigurator().apply(cElevatorMotorConfig);
    }


    @Override
    public void periodic() {
        if (!b_locked && !isFinished()) {
            double speed = (getTargetLevel().value - getEncoderValue()) / 200; // Set speed to encoder difference
            speed = Math.min(Math.max(speed, -0.125), 0.125); // Clamp speed
            //m_LeftElevator.set(speed);
            SmartDashboard.putNumber("Elevator - Speed", speed); //Elevator speed in smartdashboard
        }
        else {
            m_LeftElevator.set(0);
        }
        SmartDashboard.putBoolean("Elevator - Finished", isFinished());
        SmartDashboard.putString("Elevator - Target Level", getTargetLevel().name());
        SmartDashboard.putString("Elevator - Current Level", getElevatorLevel().name());
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
        return m_LeftElevator.getPosition().getValueAsDouble();
    }

    /**
     * Returns the closest reef level (1-Barge).
     * @return
     */
    public Level getElevatorLevel() {
        double v = getEncoderValue();
        double distance = 999999;
        Level closest = Level.Stow;
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
