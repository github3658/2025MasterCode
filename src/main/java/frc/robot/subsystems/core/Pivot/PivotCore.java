package frc.robot.subsystems.core.Pivot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Config;
import frc.robot.subsystems.hardware.Motor3658.Motor;
import frc.robot.subsystems.hardware.Motor3658.MotorBuilder;

public class PivotCore {
    private final Motor _motor = new MotorBuilder()
        .setNeutralMode(NeutralModeValue.Brake)
        .build(Config.kPivotMotorId, Config.kCanbus)
        .useEncoder(Config.kPivotEncoderId)
        .setSupplyCurrentLimit(2.5)
        .setMaxSpeed(0.1)
        .setPositionRange(0.004);

    private PivotTargets _pivotTarget;

    public Motor getMotor() { return _motor; }

    /** Will be safe if the endofactor is move up enough to clear the bar when the elevator runs. */
    public boolean isSafe() {
        return (_motor.getCurrentPosition() >= PivotTargets.SafetyTarget.value - _motor.getPositionRange());
    }

    public boolean isFinished() {
        return _motor.isAtPosition();
    }

    public PivotCore setPivot(PivotTargets target) {
        _pivotTarget = target;
        _motor.setTargetPosition(target.value);
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

    // public PivotCore setCanGoToCoralPos(boolean value) {
    //     //private booean canGoToCorelPost = false;
    //     return this;
    // }

    // public boolean getCanGoToCoralPos() {
    //     return true;
    // }
}
