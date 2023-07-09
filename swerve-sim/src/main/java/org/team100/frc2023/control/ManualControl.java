package org.team100.frc2023.control;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ManualControl {
    double rotSpeed();
    double ySpeed();
    double xSpeed();
    Trigger topButton();
}
