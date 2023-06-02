package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Illustrates multiple measurement sources. */
public class EKFMultiCorrectTest {
    static final double kDelta = 0.001;
    static final double kDt = 0.02;

    /**
     * The derivative of state.
     * 
     * x = (position, velocity)
     * xdot = (velocity, control)
     */
    Matrix<N2, N1> f(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
    }

    /**
     * Both measurements: (position, velocity)
     */
    Matrix<N2, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return x;
    }

    final Vector<N2> stateStdDevs = VecBuilder.fill(0.015, 0.17);
    final Vector<N2> measurementStdDevs = VecBuilder.fill(0.01, 0.1);
    final ExtendedAngleEstimator observer = new ExtendedAngleEstimator(
            this::f,
            this::h,
            stateStdDevs,
            measurementStdDevs,
            kDt);

    @Test
    public void testMultipleSensors() {
        observer.reset();

        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertEquals(-3.132, observer.getXhat(0), kDelta);
        assertEquals(0, observer.getXhat(1), kDelta);

        double u = -12;

        observer.predictState(u, kDt);
        observer.correctAngle(-3.134);
        observer.correctVelocity(-0.240);
        assertEquals(-3.134, observer.getXhat(0), kDelta);
        assertEquals(-0.240, observer.getXhat(1), kDelta);

        observer.predictState(u, kDt);
        observer.correctAngle(-3.141);
        observer.correctVelocity(-0.480);
        assertEquals(-3.141, observer.getXhat(0), kDelta);
        assertEquals(-0.480, observer.getXhat(1), kDelta);

        observer.predictState(u, kDt);
        observer.correctAngle(3.13);
        observer.correctVelocity(-0.720);
        assertEquals(3.130, observer.getXhat(0), kDelta);
        assertEquals(-0.720, observer.getXhat(1), kDelta);

        observer.predictState(u, kDt);
        observer.correctAngle(3.113);
        observer.correctVelocity(-0.960);
        assertEquals(3.113, observer.getXhat(0), kDelta);
        assertEquals(-0.960, observer.getXhat(1), kDelta);
    }
}
