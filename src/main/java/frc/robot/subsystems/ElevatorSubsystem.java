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
import frc.robot.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    // Up and down, reads limit switches
    // 3 pre-programmed stages accessible by button panel
    // Adjusts swerve speed for safety

    private TalonFX m_LeftElevatorMotor = new TalonFX(Config.kLeftElevatorMotor, Config.kCanbus);
    private TalonFX m_RightElevatorMotor = new TalonFX(Config.kRightElevatorMotor, Config.kCanbus);
    private DigitalInput dio_LimitSwitch = new DigitalInput(Config.kElevatorLimitSwitch);

    //region Level
    public enum Level {
        Stow(0),
        Coral1(0),
        Coral2(16),
        Coral3(52),
        Coral4(124),
        Algea1(54),
        Algea2(87),
        LevelBarge(124);
        public final double value;
        Level(double value) {
            this.value = value;
        }
    }
    //endregion
    // private final double c_AcceptableEncoderRange = 0.75;
    private final double c_AcceptableEncoderRange = 0.85;

    private boolean b_locked;
    private Level l_TargetLevel = Level.Stow;

    /**
     * Constructs the Elevator Subsystem
     */
    public ElevatorSubsystem() {
        m_LeftElevatorMotor.setPosition(0);
        m_RightElevatorMotor.setPosition(0);
        setElevatorConfig();
        m_RightElevatorMotor.setControl(new Follower(Config.kLeftElevatorMotor, false));
    }
    //region ElevatorConfig
    private void setElevatorConfig() {
        TalonFXConfiguration cElevatorMotorConfig = new TalonFXConfiguration();
        cElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_LeftElevatorMotor.getConfigurator().apply(cElevatorMotorConfig);
        m_RightElevatorMotor.getConfigurator().apply(cElevatorMotorConfig);
    }
    //endregion

    double leftSupply;
    double rightSupply;
    double d_ElevatorCurrentLimit_down = 53.0;
    double d_ElevatorCurrentLimit_up = 70.0;
    double d_ElevatorSpeedRamp = 0.0;
    //region Periodic
    @Override
    public void periodic() {
        if (!b_locked) {
            double speed = (getTargetLevel().value - getEncoderValue()) * 0.10; // Set speed to encoder difference
            if (getTargetLevel().value < getEncoderValue()) {
                speed = (getTargetLevel().value - getEncoderValue()) * 0.05;
            }
            d_ElevatorSpeedRamp = Math.min(d_ElevatorSpeedRamp + 0.1, 1.0);
            speed = Math.min(Math.max(speed, -0.75 * d_ElevatorSpeedRamp), 0.75 * d_ElevatorSpeedRamp); // Clamp speed
            if (!dio_LimitSwitch.get()) {
                m_LeftElevatorMotor.set(0);
                //m_RightElevatorMotor.set(0);
            }
            else {
                m_LeftElevatorMotor.set(speed);
                //m_RightElevatorMotor.set(speed);
            }
        }
        else {
            m_LeftElevatorMotor.set(0);
        }
        //endregion
        //region SmartDashboard
        SmartDashboard.putNumber("Elevator Position", m_LeftElevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator - Finished", isFinished());
        SmartDashboard.putBoolean("Elevator - Locked", getLocked());
        SmartDashboard.putString("Elevator - Level TARGET", getTargetLevel().name());
        SmartDashboard.putString("Elevator - Level CURRENT", getElevatorLevel().name());
        SmartDashboard.putNumber("Elevator - Encoder Error", getTargetLevel().value - getEncoderValue());
        double l = m_LeftElevatorMotor.getSupplyCurrent().getValueAsDouble();
        //endregion
        //region Safety
        if (Math.abs(l) > Math.abs(leftSupply)) {
            leftSupply = l;
            SmartDashboard.putNumber("Elevator - Left Supply", leftSupply);
            System.out.println("left "+leftSupply);
            if ((isMovingUp() && leftSupply > d_ElevatorCurrentLimit_up) || (!isMovingUp() && leftSupply > d_ElevatorCurrentLimit_down)) { // && getElevatorLevel() == Level.Stow && getEncoderValue() > getTargetLevel().value) {
                // m_LeftElevatorMotor.setPosition(0, 0);
                // m_RightElevatorMotor.setPosition(0, 0);
                // leftSupply = 0;
                currentLimitSafety();

            }
        }   
        double r = m_RightElevatorMotor.getSupplyCurrent().getValueAsDouble();
        if (Math.abs(r) > Math.abs(rightSupply)) {
            SmartDashboard.putNumber("Elevator - Right Supply", rightSupply);
            rightSupply = r;
            System.out.println("right "+rightSupply);
            if ((isMovingUp() && rightSupply > d_ElevatorCurrentLimit_up) || (!isMovingUp() && rightSupply > d_ElevatorCurrentLimit_down)) { // && getElevatorLevel() == Level.Stow && getEncoderValue() > getTargetLevel().value) {
                // m_LeftElevatorMotor.setPosition(0, 0);
                // m_RightElevatorMotor.setPosition(0, 0);
                // rightSupply = 0;
                currentLimitSafety();
            }
        }
    }
    //endregion
    /**
     * Given an elevator level, will set the elevator subsystem to that level.
     * @param level The level to set.
     */
    //region Level
    public void setLevel(Level level) {
        d_ElevatorSpeedRamp = 0;
        l_TargetLevel = level;
    }
    //endregion
    /**
     * Returns the value of the elevator motor's encoder.
     * @return
     */
    //region EncoderValue
    public double getEncoderValue() {
        //double target = getTargetLevel().value;
        double left = m_LeftElevatorMotor.getPosition().getValueAsDouble();
        //double right = m_RightElevatorMotor.getPosition().getValueAsDouble();
        //if (Math.abs(target - left) < Math.abs(target - right)) {
            return left;
        //}
        //else {
        //    return right;
        //}
    }
    //endregion
    /**
     * Returns the closest reef level (1-Barge).
     * @return
     */
    //region Elevator Level
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
    //endregion
    /**
     * Returns the reef level that the elevator is trying to go to (1-5).
     * @return
     */
    //region Target
    public Level getTargetLevel() {
        return l_TargetLevel;
    }
    //endregion
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
    //region Lock
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
    //endregion Lock
    //region Safety-Extra
    public boolean isMovingUp() {
        if (l_TargetLevel.value > getEncoderValue()) {
            return true;
        }
        else {
            return false;
        }
    }

    public void currentLimitSafety() {
        boolean b_GoingUp = isMovingUp();
        leftSupply = 0;
        rightSupply = 0;
        if (b_GoingUp) {
            System.out.println("Going UP");
            Logger.writeString("Elevator Safety (UP)", "Elevator Safety (UP)");
            // switch(getElevatorLevel()) {
            //     case Stow : 
            //         l_TargetLevel = Level.Stow;
            //         break;
            //     case Coral1: 
            //         l_TargetLevel = Level.Stow;
            //         break;
            //     case Coral2:
            //         l_TargetLevel = Level.Coral1;
            //         break;
            //     case Coral3:
            //     case Algea1:
            //         l_TargetLevel = Level.Coral2;
            //         break;
            //     case Coral4: 
            //     case Algea2:
            //         l_TargetLevel = Level.Coral3;
            //         break;
            //     case LevelBarge:
            //         l_TargetLevel = Level.Coral4;
            //         break;
            // }
        }
        
        else {
            System.out.println("Going DOWN");
            Logger.writeString("Elevator Safety (DOWN)", "Elevator Safety (DOWN)");
            switch(getElevatorLevel()) {
                case Stow : 
                    m_LeftElevatorMotor.setPosition(0, 0);
                    m_RightElevatorMotor.setPosition(0, 0);
                    rightSupply = 0;
                    leftSupply = 0;
                    break;
                default:
                    break;
                // case Coral1: 
                //     l_TargetLevel = Level.Coral2;
                //     break;
                // case Coral2:
                //     l_TargetLevel = Level.Coral3;
                //     break;
                // case Coral3:
                // case Algea1:
                //     l_TargetLevel = Level.Coral4;
                //     break;
                // case Coral4: 
                // case Algea2:
                //     l_TargetLevel = Level.LevelBarge;
                //     break;
                // case LevelBarge:
                //     l_TargetLevel = Level.LevelBarge;
                //     break;
            }
        }
        System.out.println("Retreat to "+l_TargetLevel.name());
        Logger.writeString("Retreat to", l_TargetLevel.name());
    }
}
//endregion