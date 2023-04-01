package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

public class ExampleCommand extends FastCommand {
    private final ExampleSubsystem m_subsystem;

    public ExampleCommand(ExampleSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute10ms() {
        m_subsystem.doTheThing();
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
