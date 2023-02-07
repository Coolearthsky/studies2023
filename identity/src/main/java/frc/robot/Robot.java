
package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {
    System.out.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
    System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
    System.out.printf("Identity: %s\n", Identity.get().name());
  }
}
