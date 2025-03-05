package frc.robot.subsystems.core.Pivot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Config;
import frc.robot.subsystems.hardware.Motor3658.Motor;
import frc.robot.subsystems.hardware.Motor3658.MotorBuilder;

public class PivotCore {
    private final Motor _motor = new MotorBuilder()
        .setNeutralMode(NeutralModeValue.Brake)
        .build(Config.kPivotMotorId, Config.kPivotEncoderId, Config.kCanbus)
        .setSupplyCurrentLimit(2.5)
        .setMaxSpeed(0.0625)
        .setPositionRange(0.004);

    private PivotTargets _pivotTarget;
    private boolean _hasCoral;

    public Motor getMotor() { return _motor; }

    /** Will be safe if the endofactor is move up enough to clear the bar when the elevator runs. */
    public boolean isSafe() {
        return(_motor.getCurrentPosition() >= PivotTargets.SafetyTarget.value - _motor.getPositionRange());
    }

    public boolean isFinished() {
        return(_motor.getCurrentPosition() >= getCurrentTarget().value - _motor.getPositionRange() 
            && _motor.getCurrentPosition() <= getCurrentTarget().value + _motor.getPositionRange());
    }

    public PivotCore setPivot(PivotTargets target) {
        _pivotTarget = target;
        return this;
    }

    public PivotCore adjustCurrentPosition(double offset) {
        _pivotTarget.value += offset;
        _pivotTarget.value = Math.max(_pivotTarget.value, -0.415);
        return this;
    }

    public boolean isAtTarget(PivotTargets target) {
        return _pivotTarget == target;
    }

    public PivotTargets getCurrentTarget() {
        return _pivotTarget;
    }

    public boolean hasCoral() {
        return _hasCoral;
    }
}
