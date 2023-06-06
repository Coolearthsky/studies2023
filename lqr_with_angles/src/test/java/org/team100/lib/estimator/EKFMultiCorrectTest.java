package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.NormalDoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Illustrates multiple measurement sources. */
public class EKFMultiCorrectTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    @Test
    public void testMultipleSensors() {
        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);

        Matrix<N2, N1> xhat = VecBuilder.fill(-1.0 * Math.PI + 0.01, 0);
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        Matrix<N1, N1> u = VecBuilder.fill(-12);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(-3.134), system.position());
        xhat = estimator.correct(xhat, VecBuilder.fill(-0.240), system.velocity());
        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.get(1, 0), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(-3.141), system.position());
        xhat = estimator.correct(xhat, VecBuilder.fill(-0.480), system.velocity());
        assertEquals(-3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.get(1, 0), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.13), system.position());
        xhat = estimator.correct(xhat, VecBuilder.fill(-0.720), system.velocity());
        assertEquals(3.130, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.113), system.position());
        xhat = estimator.correct(xhat, VecBuilder.fill(-0.960), system.velocity());
        assertEquals(3.113, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);
    }
}
