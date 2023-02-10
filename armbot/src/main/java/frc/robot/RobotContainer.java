package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
    private final ArmSubsystem m_robotArm = new ArmSubsystem();

    CommandXboxController m_driverController = new CommandXboxController(0);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Move the arm to 2 radians above horizontal when the 'A' button is pressed.
        m_driverController.a().onTrue(Commands.runOnce(
                () -> {
                    m_robotArm.setGoal(2);
                    m_robotArm.enable();
                },
                m_robotArm));

        // Move the arm to neutral position when the 'B' button is pressed.
        m_driverController.b().onTrue(Commands.runOnce(
                () -> {
                    m_robotArm.setGoal(ArmSubsystem.kArmOffsetRads);
                    m_robotArm.enable();
                },
                m_robotArm));

        // Disable the arm controller when Y is pressed.
        m_driverController.y().onTrue(Commands.runOnce(m_robotArm::disable));
    }

    /**
     * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be
     * called on robot
     * disable to prevent integral windup.
     */
    public void disablePIDSubsystems() {
        m_robotArm.disable();
    }
}
