package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToWaypoint extends CommandBase {
    private static final double kPeriodS = 0.02;

    private static final double kMaxSpeedM_s = 6.0;
    private static final double kMaxAccelM_s_s = 24.0;
    TrapezoidProfile.Constraints xyConstraints = new TrapezoidProfile.Constraints(kMaxSpeedM_s, kMaxAccelM_s_s);

    private static final double kMaxAngularSpeedRad_s = 6.0 * Math.PI;
    private static final double kMaxAngularAccelRad_s_s = 24.0 * Math.PI;
    TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRad_s,
            kMaxAngularAccelRad_s_s);

    private final Supplier<Pose2d> waypointSupplier;
    private final Supplier<Pose2d> poseSupplier;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<SwerveModuleState[]> outputModuleStateConsumer;

    // these try to reach the profile
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    private Pose2d previousPose = null;

    /**
     * @param waypointSupplier          the destination
     * @param poseSupplier              current robot pose from estimator
     * @param outputModuleStateConsumer control output
     * @param requirement               for the scheduler
     */
    public DriveToWaypoint(
            Supplier<Pose2d> waypointSupplier,
            Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics,
            Consumer<SwerveModuleState[]> outputModuleStateConsumer,
            Drivetrain requirement) {
        this.waypointSupplier = waypointSupplier;
        this.poseSupplier = poseSupplier;
        this.kinematics = kinematics;
        this.outputModuleStateConsumer = outputModuleStateConsumer;
        addRequirements(requirement);

        xController = new PIDController(10, 0, 0);
        yController = new PIDController(10, 0, 0);
        thetaController = new PIDController(10, 0, 0);

    }

    @Override
    public void execute() {
        final double goalSpeed = 0.0; // for now goals are always stationary.
        Pose2d waypoint = waypointSupplier.get();
        //System.out.println(waypoint);
        Pose2d pose = poseSupplier.get();
        //System.out.println(pose);

        if (previousPose == null) {
            // first time, velocity is zero
            previousPose = pose;
        }

        // for now use "city block metric". TODO: correct for diagonals

        // X
        TrapezoidProfile.State xGoal = new TrapezoidProfile.State(waypoint.getX(), goalSpeed);
        double xMeasurement = pose.getX();
        double xSpeed = xMeasurement - previousPose.getX();
        TrapezoidProfile.State xInitial = new TrapezoidProfile.State(xMeasurement, xSpeed);
        TrapezoidProfile xProfile = new TrapezoidProfile(xyConstraints, xGoal, xInitial);
        TrapezoidProfile.State xSetpoint = xProfile.calculate(kPeriodS);
        double xOutput = xController.calculate(xMeasurement, xSetpoint.position);

        // Y
        TrapezoidProfile.State yGoal = new TrapezoidProfile.State(waypoint.getY(), goalSpeed);
        double yMeasurement = pose.getY();
        double ySpeed = yMeasurement - previousPose.getY();
        TrapezoidProfile.State yInitial = new TrapezoidProfile.State(yMeasurement, ySpeed);
        TrapezoidProfile yProfile = new TrapezoidProfile(xyConstraints, yGoal, yInitial);
        TrapezoidProfile.State ySetpoint = yProfile.calculate(kPeriodS);
        double yOutput = yController.calculate(yMeasurement, ySetpoint.position);

        // THETA
        TrapezoidProfile.State thetaGoal = new TrapezoidProfile.State(waypoint.getRotation().getRadians(), goalSpeed);
        double thetaMeasurement = pose.getRotation().getRadians();
        double thetaSpeed = yMeasurement - previousPose.getRotation().getRadians();
        TrapezoidProfile.State thetaInitial = new TrapezoidProfile.State(thetaMeasurement, thetaSpeed);
        TrapezoidProfile thetaProfile = new TrapezoidProfile(thetaConstraints, thetaGoal, thetaInitial);
        TrapezoidProfile.State thetaSetpoint = thetaProfile.calculate(kPeriodS);
        double thetaOutput = thetaController.calculate(thetaMeasurement, thetaSetpoint.position);

        System.out.printf("%5.3f %5.3f %5.3f\n", xOutput, yOutput, thetaOutput);

        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, thetaOutput, pose.getRotation());
        SwerveModuleState[] targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);
        outputModuleStateConsumer.accept(targetModuleStates);
    }

    @Override
    public boolean isFinished() {
        return false; // controllers at goals, measurements at setpoints
    }
}
