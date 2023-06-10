package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class PointEstimatorTest {
    private static final double kDelta = 0.001;

    public static class Thing {

        /** xdot for x */
        public RandomVector<N2> f(RandomVector<N2> x, Matrix<N1, N1> u) {
            return x;
        }

        /** y for x */
        public RandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
            return x;
        }

        /** x for y */
        public RandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
            return y;
        }
    }

    @Test
    public void testStateForFullMeasurement() {
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>();

        // measurement is 1,0
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        // full measurement has low variance for every row
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.01);
        yP.set(1, 1, 0.01);
        RandomVector<N2> y = new RandomVector<>(yx, yP);

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        Thing thing = new Thing();
        RandomVector<N2> xhat = pointEstimator.stateForMeasurement(u, y, thing::hinv);
        // since the state is just the measurement,
        // you get the specified mean and variance of the measurement.
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.01, 0, 0, 0.01 }, xhat.P.getData(), kDelta);
    }

    @Test
    public void testStateForPartialMeasurement() {
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>();

        // measurement is 1,x
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        // partial measurement has "don't know" values
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.01);
        yP.set(1, 1, 1e9); // enormous variance; TODO: how big shouild this be?
        RandomVector<N2> y = new RandomVector<>(yx, yP);

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        Thing thing = new Thing();
        RandomVector<N2> xhat = pointEstimator.stateForMeasurement(u, y, thing::hinv);
        // since the state is just the measurement,
        // you get the specified mean and variance of the measurement.
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.01, 0, 0, 1e9 }, xhat.P.getData(), kDelta);
    }

}
