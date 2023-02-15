package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveManually extends CommandBase {
    private static final boolean fieldRelative = true;
    private final Drivetrain m_swerve;
    private final XboxController m_controller = new XboxController(0);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    // private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    // private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public DriveManually(Drivetrain m_swerve) {
        this.m_swerve = m_swerve;
        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        // final var xSpeed = -m_xspeedLimiter.calculate(getXSpeedInput1_1()) * Drivetrain.kMaxSpeed;
        final var xSpeed = -getXSpeedInput1_1() * Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        //final var ySpeed = -m_yspeedLimiter.calculate(getYSpeedInput1_1()) * Drivetrain.kMaxSpeed;
        final var ySpeed = -getYSpeedInput1_1() * Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        // final var rot = -m_rotLimiter.calculate(getRotSpeedInput1_1()) * Drivetrain.kMaxAngularSpeed;
        final var rot = -getRotSpeedInput1_1() * Drivetrain.kMaxAngularSpeed;

        m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    // more expo works well with less deadband.

    private double getRotSpeedInput1_1() {
        return expoInput(deadband(m_controller.getLeftX(), 0.01), 0.5);
    }

    private double getYSpeedInput1_1() {
        return expoInput(deadband(m_controller.getRightX(), 0.01), 0.5);
    }

    private double getXSpeedInput1_1() {
        return expoInput(deadband(m_controller.getRightY(), 0.01), 0.5);
    }

    /**
     * Mix in the cube of the input; this feature is generally called "expo"
     * in the RC community even though it's not an exponential function.
     * 
     * @param fraction how much cubic to add, [0,1]
     */
    private double expoInput(double input, double fraction) {
        return (1 - fraction) * input + fraction * input * input * input;
    }

    private double deadband(double input, double threshold) {
        return MathUtil.applyDeadband(input, threshold, Double.MAX_VALUE);
    }

}
