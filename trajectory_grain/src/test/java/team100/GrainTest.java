package team100;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;

public class GrainTest {

    private static TrajectoryConfig getConfig() {
        TrajectoryConfig config = new TrajectoryConfig(
                1.0, // max velocity 1 m/s
                1.0); // max acceleration 1 m/s/s
        config.setStartVelocity(0); // zero starting velocity
        config.setEndVelocity(0); // zero ending velocity
        return config;
    }

    @Test
    public void testGrain() {
        ControlVectorList controlVectors = new ControlVectorList();
        // at origin, pointing down x
        controlVectors.add(new Spline.ControlVector(
                new double[] { 0.0, 1.0, 0.0 },
                new double[] { 0.0, 0.0, 0.0 }));
        // at (1,0), pointing down x, no curve
        controlVectors.add(new Spline.ControlVector(
                new double[] { 1.0, 1.0, 0.0 },
                new double[] { 0.0, 0.0, 0.0 }));
        Trajectory t = TrajectoryGenerator.generateTrajectory(controlVectors,getConfig());
        for (double time = 0; time < t.getTotalTimeSeconds(); time += 0.005) {
            Trajectory.State s = t.sample(time);
            System.out.printf("%5.3f %5.3f %5.3f\n", time, s.poseMeters.getX(), s.poseMeters.getY());
        }
    }
}
