package org.team100.frc2023.control;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Control for xbox-style, e.g. Logitech F310 */
public class XboxControl implements ManualControl {
    private final CommandXboxController m_controller = new CommandXboxController(0);


    @Override
    public double rotSpeed() {
        return m_controller.getHID().getLeftX();
    }

    @Override
    public double ySpeed() {
        return m_controller.getHID().getRightX();
    }

    @Override
    public double xSpeed() {
        return m_controller.getHID().getRightY();
    }

    @Override
    public Trigger topButton() {
        return m_controller.y();
    }
    
}
