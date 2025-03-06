package frc.robot.subsystems.core.Delivery;

public enum CoralIntakeState {
    Inactive(0),
    Timer(1),
    Position(2),
    Ejecting(3);
    int value;
    private CoralIntakeState(int value) {
        this.value = value;
    }
}
