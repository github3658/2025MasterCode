package frc.robot.subsystems.hardware.Motor3658;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Motor {
    private final TalonFX _motor;
    private final CANcoder _encoder;

    private final int _motorDeviceId;

    private double _maxSpeed = 0.0625;
    private double _supplyCurrentLimit = 0.0;
    private double _targetPostition;
    private double _motorPositionRange = 0.0;

    Motor(int deviceId, String canbus) {
        _motorDeviceId = deviceId;
        _motor = new TalonFX(deviceId, canbus);
        _encoder = null;
    }

    Motor(int motorDeviceId, int endcoderDeviceId, String canbus) {
        _motorDeviceId = motorDeviceId;
        _motor = new TalonFX(motorDeviceId, canbus);
        _encoder = new CANcoder(endcoderDeviceId, canbus);
    }

    TalonFX getMotor() {
        return _motor;
    }

    /** Will zero out the motors postiion. Not usable if using an encoder to track position. */
    public Motor zeroPosition() {
        _motor.setPosition(0);
        return this;
    }

    public double getTargetPosition() {
        return _targetPostition;
    }

    public Motor setTargetPosition(double position) {
        _targetPostition = position;
        return this;
    }

    /** Will return the encoder position if it's not null, otherwise the motor position. */
    public double getCurrentPosition() {
        if (_encoder != null) return _encoder.getPosition().getValueAsDouble();
        else return _motor.getPosition().getValueAsDouble();
    }

    public double getSupplyCurrentLimit() {
        return _supplyCurrentLimit;
    }

    public Motor setSupplyCurrentLimit(double currentLimit) {
        _supplyCurrentLimit = currentLimit;
        return this;
    }

    public double getSupplyCurrent() {
        return _motor.getSupplyCurrent().getValueAsDouble();
    }

    public Motor setPositionRange(double range) {
        _motorPositionRange = range;
        return this;
    }

    public double getPositionRange() {
        return _motorPositionRange;
    }

    public Motor runToPosition(double multiplier) {
        _motor.set(getAutoSpeed(getTargetPosition(), getCurrentPosition(), multiplier));
        return this;
    }

    public double getAutoSpeed(double targetPosition, double currentPosition, double multiplier) {
        if (_supplyCurrentLimit > 0 && getSupplyCurrent() > _supplyCurrentLimit) {
            setTargetPosition(getCurrentPosition());
            SmartDashboard.putNumber("Current deviceId:="+_motorDeviceId, getSupplyCurrent());
            return 0;
        } else {       
            return Math.min(Math.max((getTargetPosition() - getCurrentPosition()) * multiplier, -_maxSpeed), _maxSpeed);
        }
    }

    public Motor setMaxSpeed(double speed) {
        _maxSpeed = speed;
        return this;
    }

    public Motor setTeleopSpeed(double speed) {
        if (_supplyCurrentLimit > 0 && getSupplyCurrent() > _supplyCurrentLimit) {
            _motor.set(0);
        } else {
            _motor.set(Math.min(speed, _maxSpeed));
        }
        return this;
    }
}
