package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Illustrates multiple measurement sources. */
public class EKFMultiCorrectTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    private RandomVector<N2> yPosition(double yd) {
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, yd); // position
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.1); // TODO: pass variance somehow
        yP.set(1, 1, 1e9); // velocity gets "don't know" variance
        return new AngularRandomVector<>(yx, yP);
    }

    private RandomVector<N2> yVelocity(double yd) {
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(1, 0, yd); // velocity
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 1e9); // position gets "don't know" variance
        yP.set(1, 1, 0.1); // TODO: pass variance somehow
        return new AngularRandomVector<>(yx, yP);
    }

    @Test
    public void testMultipleSensors() {
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1, N2> predictor = new IntegratingPredictor<>(system);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        Matrix<N1, N1> u = VecBuilder.fill(-12);

        RandomVector<N2> x;

        xhat = predictor.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(yPosition(-3.134));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(yVelocity(-0.240));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(yPosition(-3.141));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(yVelocity(-0.480));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.141, -0.480 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(yPosition(3.13));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(yVelocity(-0.720));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { 3.130, -0.720 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(yPosition(3.113));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(yVelocity(-0.960));
        xhat = pooling.fuse(x, xhat);

        assertArrayEquals(new double[] { 3.113, -0.960 }, xhat.x.getData(), kDelta);
    }

    @Test
    public void testMultipleSensorsWithTrend() {
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1, N2> predictor = new IntegratingPredictor<>(system);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);
        TrendEstimator<N2, N1, N2> trendEstimator = new TrendEstimator<>(system);

        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);

        Matrix<N1, N1> u = VecBuilder.fill(-12);

        RandomVector<N2> x;

        xhat = predictor.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(yPosition(-3.134));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(yVelocity(-0.240));
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(yPosition(-3.141));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(yVelocity(-0.480));
        xhat = pooling.fuse(x, xhat);
        // trend the past two observations
        x = trendEstimator.stateForMeasurementPair(u, yPosition(-3.134), yPosition(-3.141), kDt);
        // what does the trend say?
        assertArrayEquals(new double[] { 0, -0.350 }, x.x.getData(), kDelta);
        //
        //
        // weight 500 is very high, no wonder it doesn't have any effect.
        //
        //
        assertArrayEquals(new double[] { 1e9, 0, 0, 500 }, x.P.getData(), kDelta);

        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { -3.141, -0.480 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(yPosition(3.13));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(yVelocity(-0.720));
        xhat = pooling.fuse(x, xhat);
        // trend the past two observations
        x = trendEstimator.stateForMeasurementPair(u, yPosition(-3.141), yPosition(3.13), kDt);
        assertArrayEquals(new double[] { 0, -0.609 }, x.x.getData(), kDelta);
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { 3.130, -0.720 }, xhat.x.getData(), kDelta);

        xhat = predictor.predict(xhat, u, kDt);

        x = pointEstimator.stateForMeasurementWithZeroU(yPosition(3.113));
        xhat = pooling.fuse(x, xhat);
        x = pointEstimator.stateForMeasurementWithZeroU(yVelocity(-0.960));
        xhat = pooling.fuse(x, xhat);
        // trend the past two observations
        x = trendEstimator.stateForMeasurementPair(u, yPosition(3.13), yPosition(3.113), kDt);
        assertArrayEquals(new double[] { 0, -0.850 }, x.x.getData(), kDelta);
        xhat = pooling.fuse(x, xhat);
        assertArrayEquals(new double[] { 3.113, -0.960 }, xhat.x.getData(), kDelta);
    }
}
