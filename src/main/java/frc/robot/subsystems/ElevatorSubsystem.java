package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class ElevatorSubsystem extends SubsystemBase {
    // Up and down, reads limit switches
    // 3 pre-programmed stages accessible by button panel
    // Adjusts swerve speed for safety

    private TalonFX m_LeftElevatorMotor = new TalonFX(Config.kLeftElevatorMotor, Config.kCanbus);
    private TalonFX m_RightElevatorMotor = new TalonFX(Config.kRightElevatorMotor, Config.kCanbus);
    private DigitalInput dio_LimitSwitch = new DigitalInput(Config.kLimitSwitch);


    public enum Level {
        Stow(0),
        Coral1(0),
        Coral2(16),
        Coral3(52),
        Coral4(124),
        Algea1(54),
        Algea2(87),
        LevelBarge(8);
        public final double value;
        Level(double value) {
            this.value = value;
        }
    }

    private final double c_AcceptableEncoderRange = 0.75;

    private boolean b_locked;
    private Level l_TargetLevel = Level.Stow;

    /**
     * Constructs the Elevator Subsystem
     */
    public ElevatorSubsystem() {
        m_LeftElevatorMotor.setPosition(0);
        m_RightElevatorMotor.setPosition(0);
        setElevatorConfig();
        //m_RightElevatorMotor.setControl(new Follower(Config.kLeftElevatorMotor, true));
    }

    private void setElevatorConfig() {
        TalonFXConfiguration cElevatorMotorConfig = new TalonFXConfiguration();
        cElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_LeftElevatorMotor.getConfigurator().apply(cElevatorMotorConfig);
        m_RightElevatorMotor.getConfigurator().apply(cElevatorMotorConfig);
    }

    private void Honk() {
       // m_LeftElevatorMotor.setControl()
    }


    @Override
    public void periodic() {
        if (!b_locked) {
            double speed = (getTargetLevel().value - getEncoderValue()) * 0.05; // Set speed to encoder difference
            speed = Math.min(Math.max(speed, -0.75), 0.75); // Clamp speed
            if (!dio_LimitSwitch.get()) {
                m_LeftElevatorMotor.set(0);
                m_RightElevatorMotor.set(0);
            }
            else {
                m_LeftElevatorMotor.set(speed);
                m_RightElevatorMotor.set(speed);
            }
            SmartDashboard.putNumber("Elevator - Speed", speed); //Elevator speed in smartdashboard
        }
        else {
            m_LeftElevatorMotor.set(0);
            SmartDashboard.putNumber("Elevator - Speed", 0); //Elevator speed in smartdashboard
        }
        SmartDashboard.putBoolean("Elevator - Finished", isFinished());
        SmartDashboard.putBoolean("Elevator - Locked", getLocked());
        SmartDashboard.putNumber("Elevator - Level TARGET", getTargetLevel().value);
        SmartDashboard.putNumber("Elevator - Level CURRENT", getEncoderValue());
        SmartDashboard.putBoolean("Elevator - Limit Switch", dio_LimitSwitch.get());
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
        return m_LeftElevatorMotor.getPosition().getValueAsDouble();
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
