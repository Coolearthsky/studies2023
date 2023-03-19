package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team100.Illuminator;

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
      double value = controller.getLeftTriggerAxis();
      //double value = 1.0; // all the way on is ok
      illuminator.set(value);
    } else {
      illuminator.set(0);
    }
  }

  @Override
  public void teleopExit() {
    illuminator.set(0);
  }
}
