package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  XboxController controller = new XboxController(0);

  @Override
  public void robotPeriodic() {
    double x = controller.getRightX();
    double y = controller.getRightY();
    double h = Math.hypot(x, y);
    double a = Math.atan2(y, x);
    h = h / MaxHypot.tree.floorEntry(a).getValue();
    double x2 = h * Math.cos(a);
    double y2 = h * Math.sin(a);
    System.out.printf("x %6.3f y %6.3f h %6.3f a %6.3f x2 %6.3f y2 %6.3f\n", x, y, h, a, x2, y2);
  }
}
