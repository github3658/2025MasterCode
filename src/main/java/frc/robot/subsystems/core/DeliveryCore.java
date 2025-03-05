package frc.robot.subsystems.core;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Config;
import frc.robot.subsystems.hardware.Range;
import frc.robot.subsystems.hardware.Motor3658.Motor;
import frc.robot.subsystems.hardware.Motor3658.MotorBuilder;

public class DeliveryCore {
    private final Motor _motor = new MotorBuilder()
        .setNeutralMode(NeutralModeValue.Brake)
        .build(Config.kDeliveryMotorId, Config.kCanbus);

    private final Range _range = new Range(Config.kDeliveryRangeId, Config.kCanbus)
        .setDetectionRange(0.04)
}
