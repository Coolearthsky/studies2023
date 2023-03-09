package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import team100.OnboardIndicator;

public class Robot extends TimedRobot {
    OnboardIndicator indicator;
    XboxController controller;
    ADXRS450_Gyro gyro; // for simulation

    public Robot() {
        indicator = new OnboardIndicator(8);
        controller = new XboxController(0);
        gyro = new ADXRS450_Gyro();
    }

    @Override
    public void robotInit() {
        indicator.nogo();
    }

    @Override
    public void robotPeriodic() {
        double headingDegreesNWU = -1.0 * gyro.getAngle();
        //double headingRadiansNWU = Units.degreesToRadians(headingDegreesNWU);
        //double wrappedHeadingRadiansNWU = MathUtil.angleModulus(headingRadiansNWU);
        headingDegreesNWU = 0.0;
        double headingRadiansNWU = Units.degreesToRadians(headingDegreesNWU);
        double wrappedHeadingRadiansNWU = MathUtil.angleModulus(headingRadiansNWU);
        Rotation2d heading = new Rotation2d(wrappedHeadingRadiansNWU);
        indicator.setHeading(heading);
        if (controller.getAButton())
            indicator.go();
        else if (controller.getBButton())
            indicator.nogo();
    }
}
