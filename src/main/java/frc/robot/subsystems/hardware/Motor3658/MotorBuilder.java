package frc.robot.subsystems.hardware.Motor3658;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorBuilder {
    private final TalonFXConfiguration _config;
    
    public MotorBuilder() {
        _config = new TalonFXConfiguration();
    }

    /** The state of the motor controller bridge when output is neutral or disabled. */
    public MotorBuilder setNeutralMode(NeutralModeValue value) {
        _config.MotorOutput.NeutralMode = value;
        return this;
    }

    /** Invert state of the device as seen from the front of the motor. */
    public MotorBuilder setDirection(InvertedValue value) {
        _config.MotorOutput.Inverted = value;
        return this;
    }

    /** Will apply motor configurations and initialize the motor. */
    public Motor build(int deviceId, String canbus) {
        Motor motor = new Motor(deviceId, canbus);
        motor.getMotor().getConfigurator().apply(_config);
        return motor;
    }
}
