package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;

public class IntegratingPredictorTest {
    private static final double kDelta = 0.001;

    static RandomVector<N1> v1(double x, double P) {
        Matrix<N1, N1> xV = VecBuilder.fill(x);
        Matrix<N1, N1> PV = VecBuilder.fill(P);
        return new RandomVector<>(xV, PV);
    }

    static void assert1(RandomVector<N1> v, double x, double P) {
        assertArrayEquals(new double[] { x }, v.x.getData(), kDelta);
        assertArrayEquals(new double[] { P }, v.P.getData(), kDelta);
    }

    /** xdot = 0 */
    public Matrix<N1, N1> f1Zero(Matrix<N1, N1> x, Matrix<N1, N1> u) {
        return VecBuilder.fill(0);
    }

    /** xdot = x */
    public Matrix<N1, N1> f1X(Matrix<N1, N1> x, Matrix<N1, N1> u) {
        return x;
    }

    @Test
    public void testZero() {
        IntegratingPredictor<N1, N1> p = new IntegratingPredictor<N1, N1>(this::f1Zero);
        Matrix<N1, N1> u = VecBuilder.fill(0);
        double dtS = 1;
        // if xdot is zero then the prediction is always the same as the input
        {
            RandomVector<N1> x = v1(0, 0);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 0, 0);
        }
        {
            RandomVector<N1> x = v1(1, 1);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 1, 1);
        }
        {
            RandomVector<N1> x = v1(2, 2);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 2, 2);
        }
    }

    @Test
    public void testX() {
        IntegratingPredictor<N1, N1> p = new IntegratingPredictor<N1, N1>(this::f1X);
        Matrix<N1, N1> u = VecBuilder.fill(0);
        double dtS = 1;
        {
            // if x is zero, then nothing changes
            RandomVector<N1> x = v1(0, 0);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 0, 0);
        }
        {
            // if x is one, then the prediction should be e, and it's pretty close.
            RandomVector<N1> x = v1(1, 1);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 2.708, 2.708); // should be 2.718
        }
        {
            RandomVector<N1> x = v1(2, 2);
            RandomVector<N1> v = p.predict(x, u, dtS);
            assert1(v, 5.417, 5.417);
        }
    }
}
