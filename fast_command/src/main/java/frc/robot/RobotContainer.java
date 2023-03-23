package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

public class RobotContainer {
    private final ExampleSubsystem m_exampleSubsystem;
    private final ExampleCommand m_exampleCommand;
    private final CommandXboxController m_driverController;

    public RobotContainer() {
        m_exampleSubsystem = new ExampleSubsystem();
        m_exampleCommand = new ExampleCommand(m_exampleSubsystem);
        m_driverController = new CommandXboxController(0);
        m_driverController.b().whileTrue(m_exampleCommand);
    }
}
