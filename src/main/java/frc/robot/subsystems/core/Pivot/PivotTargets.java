package frc.robot.subsystems.core.Pivot;

public enum PivotTargets {
    CoralIntake(-0.415),
    SafetyTarget(-0.366),
    Level1(-0.415),
    Level2AndLevel3(-0.366),
    Level4(-0.243),
    AlgaeIntake(0.128);
    public double value;
    PivotTargets(double value) {
        this.value = value;
    }
}
