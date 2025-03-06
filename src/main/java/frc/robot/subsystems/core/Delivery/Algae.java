package frc.robot.subsystems.core.Delivery;

import frc.robot.subsystems.hardware.Motor3658.Motor;

public class Algae {
    private final Motor _motor;

    private final double _intakeSpeed = 0.125;
    private final double _ejectSpeed = 0.5;

    public Algae(Motor motor) {
        _motor = motor;
    }

    public Algae intakAlgae() {
        _motor.setSpeed(_intakeSpeed);
        return this;
    }

    public Algae ejectAlgae() {
        _motor.setSpeed(_ejectSpeed);
        return this;
    }

    public Algae stop() {
        _motor.stop();
        return this;
    }
}
