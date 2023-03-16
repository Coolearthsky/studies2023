package team100;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;

/**
 * Demonstrates a few approaches to trajectory generation around a corner.
 */
public class TrajectoryTest {

    /**
     * Trajectory constraints:
     * 
     * * starting and ending velocity of zero
     * * max velocity and max acceleation
     * * max centripetal acceleration
     */
    private static TrajectoryConfig getConfig() {
        TrajectoryConfig config = new TrajectoryConfig(
                1.0, // max velocity 1 m/s
                1.0); // max acceleration 1 m/s/s
        config.setStartVelocity(0); // zero starting velocity
        config.setEndVelocity(0); // zero ending velocity
        config.addConstraint(new CentripetalAccelerationConstraint(1)); // max accel 1 m/s/s
        return config;
    }

    /**
     * One control vector at the corner.
     * 
     * Overshoots in x only.
     */
    @Test
    public void testWithControlVectors() {
        ControlVectorList controlVectors = new ControlVectorList();
        // at origin, pointing down x
        controlVectors.add(new Spline.ControlVector(
                new double[] { 0.0, 1.0, 0.0 },
                new double[] { 0.0, 0.0, 0.0 }));
        // at (1,0), pointing down x, no curve
        controlVectors.add(new Spline.ControlVector(
                new double[] { 1.0, 1.0, 0.0 },
                new double[] { 0.0, 0.0, 0.0 }));
        // at (1,1), pointing down y, no curve
        controlVectors.add(new Spline.ControlVector(
                new double[] { 1.0, 0.0, 0.0 },
                new double[] { 1.0, 1.0, 0.0 }));
        Trajectory t = TrajectoryGenerator.generateTrajectory(
                controlVectors, getConfig());
        for (double time = 0; time < t.getTotalTimeSeconds(); time += 0.1) {
            Trajectory.State s = t.sample(time);
            System.out.printf("%5.3f %5.3f\n", s.poseMeters.getX(), s.poseMeters.getY());
        }
    }

    /**
     * The simple approach using a waypoint at the corner.
     * 
     * Overshoots in both x and y.
     */
    @Test
    public void testWithWaypoints() {
        // at the origin, pointing down x
        Pose2d start = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        List<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        // waypoint at (1,0)
        interiorWaypoints.add(new Translation2d(1, 0));
        // at (1,1) pointing down y
        Pose2d end = new Pose2d(new Translation2d(1, 1), new Rotation2d(Math.PI / 2));
        Trajectory t = TrajectoryGenerator.generateTrajectory(
                start, interiorWaypoints, end, getConfig());
        for (double time = 0; time < t.getTotalTimeSeconds(); time += 0.1) {
            Trajectory.State s = t.sample(time);
            System.out.printf("%5.3f %5.3f\n", s.poseMeters.getX(), s.poseMeters.getY());
        }
    }

    /**
     * Two control vectors near the corner.
     * 
     * No overshoot.
     */
    @Test
    public void testWithControlVectorsWithoutOvershoot() {
        ControlVectorList controlVectors = new ControlVectorList();
        // at origin, pointing down x
        controlVectors.add(new Spline.ControlVector(
                new double[] { 0.0, 1.0, 0.0 },
                new double[] { 0.0, 0.0, 0.0 }));
        // at (0.75,0), pointing down x
        controlVectors.add(new Spline.ControlVector(
                new double[] { 0.75, 0.5, 0.0 },
                new double[] { 0.0, 0.0, 0.0 }));
        // at (1,0.25), pointing down y, no curve
        controlVectors.add(new Spline.ControlVector(
                new double[] { 1.0, 0.0, 0.0 },
                new double[] { 0.25, 0.5, 0.0 }));
        // at (1,1), pointing down y, no curve
        controlVectors.add(new Spline.ControlVector(
                new double[] { 1.0, 0.0, 0.0 },
                new double[] { 1.0, 1.0, 0.0 }));
        Trajectory t = TrajectoryGenerator.generateTrajectory(
                controlVectors, getConfig());
        for (double time = 0; time < t.getTotalTimeSeconds(); time += 0.1) {
            Trajectory.State s = t.sample(time);
            System.out.printf("%5.3f %5.3f\n", s.poseMeters.getX(), s.poseMeters.getY());
        }
    }
}
