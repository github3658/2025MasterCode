package frc.robot.subsystems.hardware.Motor3658;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorBuilder {
    private final TalonFXConfiguration _config;
    
    public MotorBuilder() {
        _config = new TalonFXConfiguration();
    }

    public MotorBuilder setNeutralMode(NeutralModeValue value) {
        _config.MotorOutput.NeutralMode = value;
        return this;
    }

    public MotorBuilder setDirection(InvertedValue value) {
        _config.MotorOutput.Inverted = value;
        return this;
    }

    public Motor build(int deviceId, String canbus) {
        Motor motor = new Motor(deviceId, canbus);
        motor.getMotor().getConfigurator().apply(_config);
        return motor;
    }

    public Motor build(int motorDeviceId, int encoderDeviceId, String canbus) {
        Motor motor = new Motor(motorDeviceId, encoderDeviceId, canbus);
        motor.getMotor().getConfigurator().apply(_config);
        return motor;
    }
}
