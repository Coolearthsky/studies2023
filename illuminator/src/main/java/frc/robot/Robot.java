package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final Illuminator illuminator = new Illuminator();
  private final XboxController controller = new XboxController(0);

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    if (controller.getAButton()) {
      illuminator.on();
    } else {
      illuminator.off();
    }
  }

  @Override
  public void teleopExit() {
    illuminator.off();
  }
}
