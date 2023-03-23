package frc.robot;

import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ExampleGlobalMeasurementSensor {
    /**
     * Get a "noisy" fake global pose reading.
     *
     * @param estimatedRobotPose The robot pose.
     */
    public static Pose2d getEstimatedGlobalPose(Pose2d estimatedRobotPose) {
        var rand = StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(5)));
        return new Pose2d(
                estimatedRobotPose.getX() + rand.get(0, 0),
                estimatedRobotPose.getY() + rand.get(1, 0),
                estimatedRobotPose.getRotation().plus(new Rotation2d(rand.get(2, 0))));
    }
}
