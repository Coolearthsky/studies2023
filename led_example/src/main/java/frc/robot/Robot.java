package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import team100.GoNoGoIndicator;

public class Robot extends TimedRobot {
    GoNoGoIndicator indicator;
    XboxController controller;

    public Robot() {
        indicator = new GoNoGoIndicator(8);
        controller = new XboxController(0);
    }

    @Override
    public void robotInit() {
        indicator.nogo();
    }

    @Override
    public void robotPeriodic() {
        if (controller.getAButton())
            indicator.go();
        else if (controller.getBButton())
            indicator.nogo();
    }
}
