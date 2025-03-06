package frc.robot.subsystems.hardware.Motor3658;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.hardware.Encoder3658.Encoder;

public class Motor {
    //region Properties
    private final TalonFX _motor;
    private Encoder _encoder = null;

    private final int _motorDeviceId;
    private final String _canbus;

    private double _maxSpeed = 0.0625;
    private double _supplyCurrentLimit = 0.0;
    private double _targetPostition;
    private double _motorPositionRange = 0.0;
    //endregion

    Motor(int deviceId, String canbus) {
        _motorDeviceId = deviceId;
        _canbus = canbus;
        _motor = new TalonFX(deviceId, canbus);
    }

    TalonFX getMotor() {
        return _motor;
    }
    
    //region SupplyCurrent
    public double getSupplyCurrentLimit() {
        return _supplyCurrentLimit;
    }

    /** When set it will stop the motor when the current limit is met or exceeded. */
    public Motor setSupplyCurrentLimit(double currentLimit) {
        _supplyCurrentLimit = currentLimit;
        return this;
    }

    public double getSupplyCurrent() {
        return _motor.getSupplyCurrent().getValueAsDouble();
    }
    //endregion

    //region Position
    /** When an encoder is defined the run to position methods will use the encoder's position instead of the motors.
     *  Will use the same canbus defined for the motor.
     */
    public Motor useEncoder(int deviceId) {
        _encoder = new Encoder(deviceId, _canbus);
        return this;
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
        if (_encoder != null) return _encoder.getPosition();
        else return _motor.getPosition().getValueAsDouble();
    }

    /** Will allow the motor to start stopping early to prevent overshooting taget. */
    public Motor setPositionRange(double range) {
        _motorPositionRange = range;
        return this;
    }

    public double getPositionRange() {
        return _motorPositionRange;
    }

    /** Will consider the motor has reached it's target if it lands within it's range. If the Position range is 0 then it has to stop on the exact target position.  */
    public boolean isAtPosition() {
        return (getCurrentPosition() >= getTargetPosition() - getPositionRange() 
            && getCurrentPosition() <= getTargetPosition() + getPositionRange());
    }
    //endregion

    //region Speeds
    /** Will be the top speed the motors are allowed to go. */
    public Motor setMaxSpeed(double speed) {
        _maxSpeed = speed;
        return this;
    }

    public Motor stop() {
        _motor.set(0);
        return this;
    }

    public double getAutonomousSpeed(double targetPosition, double currentPosition, double multiplier) {
        /* If the supply current limit is reached we need to set the target position to the current position and set the speed to 0 */
        if (isSupplyCurrentExceeded()) {
            setTargetPosition(getCurrentPosition());
            SmartDashboard.putNumber("Current deviceId:="+_motorDeviceId, getSupplyCurrent());
            return 0;
        } else {       
            return Math.min(Math.max((getTargetPosition() - getCurrentPosition()) * multiplier, -_maxSpeed), _maxSpeed);
        }
    }

    public Motor setSpeed(double speed) {
        /* If the supply current limit is reached we need to the speed to 0. This may "jitter" the subsystem until the driver stops trying to move it. */
        if (isSupplyCurrentExceeded()) {
            _motor.set(0);
        } else {
            _motor.set(Math.min(speed, _maxSpeed));
        }
        return this;
    }

    private boolean isSupplyCurrentExceeded() {
        return _supplyCurrentLimit != 0 && Math.abs(getSupplyCurrent()) > Math.abs(_supplyCurrentLimit);
    }
    //endregion
}
