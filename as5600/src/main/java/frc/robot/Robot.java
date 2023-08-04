
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private AS5600_I2C m_as5600;

    @Override
    public void robotInit() {
        m_as5600 = new AS5600_I2C();
    }

    @Override
    public void robotPeriodic() {
        System.out.println(m_as5600.get().getRadians());
    }
}
