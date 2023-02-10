package team100.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.subsystems.Arm;

/**
 * The simplest possible command: apply the input directly to the arm as motor
 * output.
 */
public class MoveArmManually extends CommandBase {
    private final DoubleSupplier speed;
    private final Arm arm;

    public MoveArmManually(DoubleSupplier speed, Arm arm) {
        this.speed = speed;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.set(speed.getAsDouble());
    }
}
