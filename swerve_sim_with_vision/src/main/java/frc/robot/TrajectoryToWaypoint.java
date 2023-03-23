package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrajectoryToWaypoint extends CommandBase {

    private final Timer m_timer = new Timer();

    private final Drivetrain m_swerve;
    private final Pose2d goal;

    private final TrajectoryConfig translationConfig;
    private final ProfiledPIDController m_rotationController;
    private final PIDController xController;
    private final PIDController yController;
    private final HolonomicDriveController m_controller;

    private Trajectory m_trajectory;

    // Network Tables

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_table = inst.getTable("robot");
    private final DoublePublisher m_rotationSetpointPosition = m_table.getDoubleTopic("rotationSetpointPosition")
            .publish();
    private final DoublePublisher m_rotationSetpointVelocity = m_table.getDoubleTopic("rotationSetpointVelocity")
            .publish();

    private final DoublePublisher m_rotationPositionError = m_table.getDoubleTopic("rotationPositionError").publish();
    private final DoublePublisher m_rotationVelocityError = m_table.getDoubleTopic("rotationVelocityError").publish();

    private final DoublePublisher m_XSetpointPub = m_table.getDoubleTopic("xSetpoint").publish();
    private final DoublePublisher m_YSetpointPub = m_table.getDoubleTopic("ySetpoint").publish();

    private final DoublePublisher m_XMeasurementPub = m_table.getDoubleTopic("xMeasurement").publish();
    private final DoublePublisher m_YMeasurementPub = m_table.getDoubleTopic("yMeasurement").publish();

    private final DoublePublisher m_XErrorPub = m_table.getDoubleTopic("xError").publish();
    private final DoublePublisher m_YErrorPub = m_table.getDoubleTopic("yError").publish();

    public TrajectoryToWaypoint(Pose2d goal, Drivetrain m_swerve) {
        this.goal = goal;
        this.m_swerve = m_swerve;
        xController = new PIDController(5, 0, 0);
        SmartDashboard.putData("x controller", xController);
        yController = new PIDController(5, 0, 0);
        SmartDashboard.putData("y controller", yController);
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(4, 4);
        m_rotationController = new ProfiledPIDController(5, 0, 0, rotationConstraints);
        SmartDashboard.putData("rotation controller", m_rotationController);
        m_controller = new HolonomicDriveController(xController, yController, m_rotationController);
        translationConfig = new TrajectoryConfig(5.0, 20.0).setKinematics(m_swerve.m_kinematics);
        addRequirements(m_swerve);
    }

    private Trajectory makeTrajectory() {
        Pose2d currentPose = m_swerve.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d goalTranslation = goal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(currentTranslation, angleToGoal),
                List.of(),
                new Pose2d(goalTranslation, angleToGoal),
                translationConfig);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_trajectory = makeTrajectory();
    }

    /**
     * Runs until the button is released.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        SwerveModuleState[] stopped = new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        m_swerve.setModuleStates(stopped);
    }

    public void execute() {
        m_swerve.updateOdometry();

        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);

        Pose2d pose = m_swerve.getPose();
        var targetChassisSpeeds = m_controller.calculate(pose, desiredState, goal.getRotation());
        var targetModuleStates = m_swerve.m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_swerve.setModuleStates(targetModuleStates);

        m_rotationSetpointPosition.set(m_rotationController.getSetpoint().position);
        m_rotationSetpointVelocity.set(m_rotationController.getSetpoint().velocity);
        m_rotationPositionError.set(m_rotationController.getPositionError());
        m_rotationVelocityError.set(m_rotationController.getVelocityError());
        m_XSetpointPub.set(xController.getSetpoint());
        m_YSetpointPub.set(yController.getSetpoint());
        m_XMeasurementPub.set(pose.getX());
        m_YMeasurementPub.set(pose.getY());
        m_XErrorPub.set(xController.getPositionError());
        m_YErrorPub.set(yController.getPositionError());

    }
}
