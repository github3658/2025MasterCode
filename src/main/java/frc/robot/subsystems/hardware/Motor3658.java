package frc.robot.subsystems.hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Motor3658 {
    private final TalonFX _motor;
    private final TalonFXConfiguration _config;

    private double _maxSpeed = 0.5;

    public Motor3658(int deviceId, String canbus) {
        _motor = new TalonFX(deviceId, canbus);
        _config = new TalonFXConfiguration();
    }

    public Motor3658 setNeutralMode(NeutralModeValue value) {
        _config.MotorOutput.NeutralMode = value;
        return this;
    }

    public Motor3658 setDirection(InvertedValue value) {
        _config.MotorOutput.Inverted = value;
        return this;
    }

    public Motor3658 setMaxSpeed(double speed) {
        _maxSpeed = speed;
        return this;
    }

    public Motor3658 build() {
        _motor.getConfigurator().apply(_config);
        return this;
    }


}
