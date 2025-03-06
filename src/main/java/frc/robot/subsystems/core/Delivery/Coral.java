package frc.robot.subsystems.core.Delivery;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;
import frc.robot.subsystems.hardware.Motor3658.Motor;
import frc.robot.subsystems.hardware.Range3658.Range;
import frc.robot.subsystems.hardware.Range3658.RangeBuilder;

public class Coral {

    private final Motor _motor;
    
    private final Range _range = new RangeBuilder()
        .setProximityThreshold(0.05)
        .setProximityHysteresis(0.01)
        .setFOVRangeX(6.75)
        .setFOVRangeY(6.75)
        .build(Config.kDeliveryRangeId, Config.kCanbus);

    private final double _intakeSpeed = 0.125;
    private final double _ejectSpeed = 0.3;

    private final Timer _intakeTimer = new Timer();
    private final double _intakeTimeout = 5.0; //seconds

    /** Once coral is detect we need to add this to the current position of the motor. */
    private final double _intakePositionTarget = 3.0;
    private final double _ejectPositionTarget = 7.5;

    private CoralIntakeState _intakeState = CoralIntakeState.Inactive;
    
    private boolean _isIntakeDisabled = false;
    private boolean _isEjectDisabled = false;

    public Coral(Motor motor) {
        _motor = motor;
    }

    public Range getRange() {
        return _range;
    }

    public boolean getIsIntakeDisabled() {
        return _isIntakeDisabled;
    }

    public boolean getIsEjectDisabled() {
        return _isEjectDisabled;
    }

    public CoralIntakeState getIntakState() {
        return _intakeState;
    }

    public Coral setIntakeState(CoralIntakeState state) {
        _intakeState = state;
        return this;
    }

    public Coral startIntake() {
        if (_intakeState == CoralIntakeState.Inactive) {
            _isIntakeDisabled = true;
            _isEjectDisabled = true;
            _motor.setSpeed(_intakeSpeed);
            _intakeTimer.reset();
            _intakeTimer.start();
            _intakeState = CoralIntakeState.Timer;
        }

        return this;
    }

    public Coral ejectCoral() {
        if (_intakeState == CoralIntakeState.Inactive) {
            _intakeState = CoralIntakeState.Ejecting;
            _motor.setTargetPosition(_motor.getCurrentPosition() + _ejectPositionTarget);
            _motor.setSpeed(_ejectSpeed);
        }
        return this;
    }

    public Coral trackTimer() {
        if (_intakeState == CoralIntakeState.Timer) {
            // We are now switching to a run to position method
            if (hasCoral()) {
                _intakeTimer.stop();
                _motor.setTargetPosition(_motor.getCurrentPosition() + _intakePositionTarget);
                _intakeState = CoralIntakeState.Position;
            } else if (_intakeTimer.hasElapsed(_intakeTimeout)) {
                _motor.stop();           
                _intakeTimer.stop();
                _intakeState = CoralIntakeState.Inactive;
                _isIntakeDisabled = true;
                _isEjectDisabled = true;
            }
        }

        return this;
    }

    public Coral runToPosition() {
        if (_intakeState == CoralIntakeState.Position || _intakeState == CoralIntakeState.Ejecting) {
            if (_motor.isAtPosition()) {
                _motor.stop();

                if (_intakeState == CoralIntakeState.Position) _isEjectDisabled = false;
                else _isIntakeDisabled = false;
                
                _intakeState = CoralIntakeState.Inactive;
                
            }
        }

        return this;
    }

    public boolean hasCoral() {
        return _range.isDetected();
    }
}
