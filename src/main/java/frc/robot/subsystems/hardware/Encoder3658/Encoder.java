package frc.robot.subsystems.hardware.Encoder3658;

import com.ctre.phoenix6.hardware.CANcoder;

public class Encoder {
    private final CANcoder _encoder;

    public Encoder(int deviceId, String canbus) {
        _encoder = new CANcoder(deviceId, canbus);
    }

    public double getAbsolutePosition() {
        return _encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getPosition() {
        return _encoder.getPosition().getValueAsDouble();
    }
}
