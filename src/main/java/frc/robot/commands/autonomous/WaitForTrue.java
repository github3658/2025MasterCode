package frc.robot.commands.autonomous;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForTrue extends Command {

    private BooleanSupplier b_Supplier;

    public WaitForTrue(BooleanSupplier b) {
        b_Supplier = b;
    }

    @Override
    public boolean isFinished() {
        return b_Supplier.getAsBoolean();
    }
}
