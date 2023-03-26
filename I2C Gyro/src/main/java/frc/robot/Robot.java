package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot implements Sendable {

  AHRS ahrs;

  public Robot() {
    ahrs = new AHRS(I2C.Port.kMXP);
    SmartDashboard.putData("demo", this);
  }

  @Override
  public void robotPeriodic() {
   // System.out.println(ahrs.getYaw());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("ahrs yaw", () -> ahrs.getYaw(), null);
  }
}
