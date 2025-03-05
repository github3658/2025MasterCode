package frc.robot.subsystems.core.Pivot;

public enum PivotEjectSpeeds {
    Level1(0.3),
    Level2(0.3),
    Level3(0.3),
    Level4(0.3);
    double value;
    private PivotEjectSpeeds(double value) {
        this.value = value;
    }
}
