package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    public static final double kMaxSpeedMS = 6.0;

    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // package visibility for testing
    final SwerveModule m_frontLeft = new SwerveModule("FrontLeft", 1, 2, 0, 1, 2, 3);
    final SwerveModule m_frontRight = new SwerveModule("FrontRight", 3, 4, 4, 5, 6, 7);
    final SwerveModule m_backLeft = new SwerveModule("BackLeft", 5, 6, 8, 9, 10, 11);
    final SwerveModule m_backRight = new SwerveModule("BackRight", 7, 8, 12, 13, 14, 15);

    final AnalogGyro m_gyro = new AnalogGyro(0);
    // note gyro is NED, robot is NWU, see inversion below.
    final AnalogGyroSim gyroSim = new AnalogGyroSim(m_gyro);

    final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private double m_prevTimeSeconds = Timer.getFPGATimestamp();
    private final double m_nominalDtS = 0.02; // Seconds

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            m_gyro.getRotation2d(), // NWU
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.001, 0.001, 0.001));

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    DoublePublisher xSpeedPubM_s = inst.getTable("desired").getDoubleTopic("xspeed m_s").publish();
    DoublePublisher ySpeedPubM_s = inst.getTable("desired").getDoubleTopic("yspeed m_s").publish();
    DoublePublisher thetaSpeedPubRad_s = inst.getTable("desired").getDoubleTopic("thetaspeed rad_s").publish();

    DoublePublisher actualXSpeedPubM_s = inst.getTable("actual").getDoubleTopic("xspeed m_s").publish();
    DoublePublisher actualYSpeedPubM_s = inst.getTable("actual").getDoubleTopic("yspeed m_s").publish();
    DoublePublisher actualThetaSpeedPubRad_s = inst.getTable("actual").getDoubleTopic("thetaspeed rad_s").publish();

    DoubleArrayPublisher robotPosePub;
    StringPublisher fieldTypePub;

    List<CallbackStore> cbs = new ArrayList<CallbackStore>();

    ChassisSpeeds speeds;

    public Drivetrain() {
        m_gyro.reset();
        inst.startClient4("blarg");
        NetworkTable fieldTable = inst.getTable("field");
        robotPosePub = fieldTable.getDoubleArrayTopic("robotPose").publish();
        fieldTypePub = fieldTable.getStringTopic(".type").publish();
        fieldTypePub.set("Field2d");
    }

    public Pose2d getPose() {
        //updateOdometry();
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(m_gyro.getRotation2d(), // NWU
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                }, pose);
    }

    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        m_frontLeft.publishState(swerveModuleStates[0]);
        m_frontRight.publishState(swerveModuleStates[1]);
        m_backLeft.publishState(swerveModuleStates[2]);
        m_backRight.publishState(swerveModuleStates[3]);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeedMS);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void updateOdometry() {
        m_poseEstimator.update(
                m_gyro.getRotation2d(), // NWU
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });

        Pose2d newEstimate = m_poseEstimator.getEstimatedPosition();
        robotPosePub.set(new double[] {
                newEstimate.getX(),
                newEstimate.getY(),
                newEstimate.getRotation().getDegrees()
        });

        m_poseEstimator.addVisionMeasurement(
                ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition()),
                Timer.getFPGATimestamp());

    }

    public void simulationInit() {
        m_frontLeft.simulationInit();
        m_frontRight.simulationInit();
        m_backLeft.simulationInit();
        m_backRight.simulationInit();
    }

    public void simulationPeriodic() {
        double currentTimeSeconds = Timer.getFPGATimestamp();
        double dtS = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDtS;
        m_prevTimeSeconds = currentTimeSeconds;
        simulationPeriodic(dtS);
    }

    public void simulationPeriodic(final double dtS) {
        m_frontLeft.simulationPeriodic(dtS);
        m_frontRight.simulationPeriodic(dtS);
        m_backLeft.simulationPeriodic(dtS);
        m_backRight.simulationPeriodic(dtS);

        // in simulation these should be the values we just set
        // in SwerveModule.simulationPeriodic(), so we don't need
        // to adjust them *again*, just use them to update the gyro.
        SwerveModuleState[] states = new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState()
        };

        // rotational velocity is correct here.
        speeds = m_kinematics.toChassisSpeeds(states);

        // finally adjust the simulator gyro.
        // the pose estimator figures out the X/Y part but it depends on the gyro.
        // since omega is the same in both coordinate schemes, just use that.
        double oldAngleDeg = gyroSim.getAngle();
        double dThetaDeg = -1.0 * new Rotation2d(speeds.omegaRadiansPerSecond * dtS).getDegrees();
        double newAngleDeg = oldAngleDeg + dThetaDeg;
        // note that the "angle" in a gyro is NED, but everything else (e.g robot pose)
        // is NWU, so invert here.
        gyroSim.setAngle(newAngleDeg);

        xSpeedPubM_s.set(speeds.vxMetersPerSecond);
        ySpeedPubM_s.set(speeds.vyMetersPerSecond);
        thetaSpeedPubRad_s.set(-1.0 * speeds.omegaRadiansPerSecond);
    }

    public void close() {
        m_frontLeft.close();
        m_frontRight.close();
        m_backLeft.close();
        m_backRight.close();
        m_gyro.close();
    }
}
