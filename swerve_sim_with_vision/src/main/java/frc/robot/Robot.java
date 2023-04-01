
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final Drivetrain m_swerve;
    private final Command autoc;

    public Robot() {
        m_swerve = new Drivetrain();
        autoc = new TrajectoryToWaypoint(new Pose2d(8, 4, new Rotation2d()), m_swerve);
    }

    @Override
    public void autonomousInit() {
        autoc.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    public void autonomousExit() {
        autoc.cancel();
    }

    @Override
    public void simulationInit() {
        m_swerve.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        m_swerve.simulationPeriodic();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
