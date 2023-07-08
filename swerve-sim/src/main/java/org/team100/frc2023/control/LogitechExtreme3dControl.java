package org.team100.frc2023.control;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Control for extreme 3D Pro joystick. */
public class LogitechExtreme3dControl implements ManualControl {
    private final CommandJoystick m_controller = new CommandJoystick(0);


    @Override
    public double rotSpeed() {
        return m_controller.getHID().getTwist();
    }

    @Override
    public double ySpeed() {
        return m_controller.getHID().getX();
    }

    @Override
    public double xSpeed() {
        return m_controller.getHID().getY();
    }

    @Override
    public Trigger topButton() {
        return m_controller.trigger();
    }
    
}
