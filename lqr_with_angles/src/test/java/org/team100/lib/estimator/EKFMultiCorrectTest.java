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

        estimator.reset();

        estimator.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertEquals(-3.132, estimator.getXhat(0), kDelta);
        assertEquals(0, estimator.getXhat(1), kDelta);

        Matrix<N1, N1> u = VecBuilder.fill(-12);

        estimator.predictState(u, kDt);
        estimator.correct(VecBuilder.fill(-3.134), system.position());
        estimator.correct(VecBuilder.fill(-0.240), system.velocity());
        assertEquals(-3.134, estimator.getXhat(0), kDelta);
        assertEquals(-0.240, estimator.getXhat(1), kDelta);

        estimator.predictState(u, kDt);
        estimator.correct(VecBuilder.fill(-3.141), system.position());
        estimator.correct(VecBuilder.fill(-0.480), system.velocity());
        assertEquals(-3.141, estimator.getXhat(0), kDelta);
        assertEquals(-0.480, estimator.getXhat(1), kDelta);

        estimator.predictState(u, kDt);
        estimator.correct(VecBuilder.fill(3.13), system.position());
        estimator.correct(VecBuilder.fill(-0.720), system.velocity());
        assertEquals(3.130, estimator.getXhat(0), kDelta);
        assertEquals(-0.720, estimator.getXhat(1), kDelta);

        estimator.predictState(u, kDt);
        estimator.correct(VecBuilder.fill(3.113), system.position());
        estimator.correct(VecBuilder.fill(-0.960), system.velocity());
        assertEquals(3.113, estimator.getXhat(0), kDelta);
        assertEquals(-0.960, estimator.getXhat(1), kDelta);
    }
}
