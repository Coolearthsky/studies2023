package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.system.examples.DoubleIntegrator1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Illustrates multiple measurement sources. */
public class EKFMultiCorrectTest {
    static final double kDelta = 0.001;
    static final double kDt = 0.02;

    final DoubleIntegrator1D system = new DoubleIntegrator1D(0.01, 0.1, 0.015, 0.17);

    final ExtendedAngleEstimator<N2, N1> observer = new ExtendedAngleEstimator<N2, N1>(Nat.N2(), Nat.N1(), system, kDt);

    @Test
    public void testMultipleSensors() {
        observer.reset();

        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertEquals(-3.132, observer.getXhat(0), kDelta);
        assertEquals(0, observer.getXhat(1), kDelta);

        Matrix<N1, N1> u = VecBuilder.fill(-12);

        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(-3.134), system.position());
        observer.correct(VecBuilder.fill(-0.240), system.velocity());
        assertEquals(-3.134, observer.getXhat(0), kDelta);
        assertEquals(-0.240, observer.getXhat(1), kDelta);

        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(-3.141), system.position());
        observer.correct(VecBuilder.fill(-0.480), system.velocity());
        assertEquals(-3.141, observer.getXhat(0), kDelta);
        assertEquals(-0.480, observer.getXhat(1), kDelta);

        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.13), system.position());
        observer.correct(VecBuilder.fill(-0.720), system.velocity());
        assertEquals(3.130, observer.getXhat(0), kDelta);
        assertEquals(-0.720, observer.getXhat(1), kDelta);

        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.113), system.position());
        observer.correct(VecBuilder.fill(-0.960), system.velocity());
        assertEquals(3.113, observer.getXhat(0), kDelta);
        assertEquals(-0.960, observer.getXhat(1), kDelta);
    }
}
