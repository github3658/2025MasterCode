package frc.robot.subsystems.core.Delivery;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Config;
import frc.robot.subsystems.hardware.Motor3658.Motor;
import frc.robot.subsystems.hardware.Motor3658.MotorBuilder;

public class DeliveryCore {
    private final Motor _motor = new MotorBuilder()
        .setNeutralMode(NeutralModeValue.Brake)
        .build(Config.kDeliveryMotorId, Config.kCanbus)
        .setSupplyCurrentLimit(5.0)
        .setMaxSpeed(0.5);

    private final Coral _coral = new Coral(_motor);

    private final Algae _algae = new Algae(_motor);

    public DeliveryCore() { }

    public Coral getCoral() {
        return _coral;
    }

    public Algae getAlgae() {
        return _algae;
    }
}
