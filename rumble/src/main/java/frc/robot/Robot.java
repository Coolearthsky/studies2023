
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  XboxController controller = new XboxController(0);

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (controller.getAButton()) {
      controller.setRumble(RumbleType.kLeftRumble, 1.0);
    } else {
      controller.setRumble(RumbleType.kLeftRumble, 0.0);
    }
    if (controller.getBButton()) {
      controller.setRumble(RumbleType.kRightRumble, 1.0);
    } else {
      controller.setRumble(RumbleType.kRightRumble, 0.0);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}



  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
   

  }


}
