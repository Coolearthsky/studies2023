package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

/** Illustrates multiple measurement sources. */
public class EKFMultiCorrectTest {
    static final double kDelta = 0.001;
    static final double kDt = 0.02;
    final Vector<N2> stateStdDevs = VecBuilder.fill(0.015, 0.17);
    final Vector<N2> measurementStdDevs = VecBuilder.fill(0.01, 0.1);
    final AngleEstimator observer = new AngleEstimator(stateStdDevs, measurementStdDevs, kDt);

    @Test
    public void testMultipleSensors() {
        observer.reset();

        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertEquals(-3.132, observer.getXhat(0), kDelta);
        assertEquals(0, observer.getXhat(1), kDelta);

        double u = -12;

        observer.predictState(u, kDt);
        observer.correctAngle(u, -3.134);
        observer.correctVelocity(u, -0.240);
        assertEquals(-3.134, observer.getXhat(0), kDelta);
        assertEquals(-0.240, observer.getXhat(1), kDelta);

        observer.predictState(u, kDt);
        observer.correctAngle(u, -3.141);
        observer.correctVelocity(u, -0.480);
        assertEquals(-3.141, observer.getXhat(0), kDelta);
        assertEquals(-0.480, observer.getXhat(1), kDelta);

        observer.predictState(u, kDt);
        observer.correctAngle(u, 3.13);
        observer.correctVelocity(u, -0.720);
        assertEquals(3.130, observer.getXhat(0), kDelta);
        assertEquals(-0.720, observer.getXhat(1), kDelta);

        observer.predictState(u, kDt);
        observer.correctAngle(u, 3.113);
        observer.correctVelocity(u, -0.960);
        assertEquals(3.113, observer.getXhat(0), kDelta);
        assertEquals(-0.960, observer.getXhat(1), kDelta);
    }
}
