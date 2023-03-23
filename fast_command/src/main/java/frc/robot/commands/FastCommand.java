package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command with faster loop in its own thread.
 * 
 * Remember to call super from initialize() and end() to start and stop the
 * thread.
 */
public abstract class FastCommand extends CommandBase {
    private static final double kDtSec = 0.01;
    private final Notifier notifier;

    public FastCommand() {
        notifier = new Notifier(this::execute10ms);
    }

    @Override
    public void initialize() {
        notifier.startPeriodic(kDtSec);
    }

    /**
     * Called by the notifier thread when the command is running.
     */
    public abstract void execute10ms();

    /**
     * The scheduler calls execute() at 20ms frequency.
     * It's final to keep subclasses from touching it.
     */
    @Override
    public final void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        notifier.stop();
    }
}
