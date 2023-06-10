package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.Sensor;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.NormalDoubleIntegratorRotary1D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class NonlinearEstimatorTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    // private AngularRandomVector<N1> y1(double yd) {
    // return new AngularRandomVector<>(VecBuilder.fill(yd), VecBuilder.fill(0.1));
    // }

    private RandomVector<N2> ypos(double yd) {

        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, yd); // position

        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.1); // TODO: pass variance somehow
        yP.set(1, 1, 1e9); // velocity gets "don't know" variance
        return new AngularRandomVector<>(yx, yP);

        // return new RandomVector<>(VecBuilder.fill(yd),VecBuilder.fill(0.1));
    }

    private RandomVector<N2> yvel(double yd) {

        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(1, 0, yd); // velocity

        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 1e9); // position gets "don't know" variance
        yP.set(1, 1, 0.1); // TODO: pass variance somehow
        return new AngularRandomVector<>(yx, yP);

        // return new RandomVector<>(VecBuilder.fill(yd),VecBuilder.fill(0.1));
    }

    @Test
    public void testObserverWrappingPredictOnly() {
        // just test the observer prediction across the boundary
        // it just predicts over and over.
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1> predictor = new IntegratingPredictor<>();
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>();
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, predictor, pointEstimator, pooling);

        // initially, state estimate: at zero, motionless
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);

        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);

        // saturate negative-going
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        xhat = estimator.predictState(xhat, u, kDt);
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        assertArrayEquals(new double[] { -3.141, -0.480 }, xhat.x.getData(), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        assertArrayEquals(new double[] { 3.130, -0.720 }, xhat.x.getData(), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        assertArrayEquals(new double[] { 3.113, -0.960 }, xhat.x.getData(), kDelta);
    }

    @Test
    public void testObserverWrappingCorrectVelocityOnly() {
        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D() {
            public Sensor<N2, N1, N2> newFull() {
                return new FullSensor() {
                    // public Matrix<N2, N1> stdev() {
                    // return VecBuilder.fill(0.1, 0.00001);
                    // }
                };
            }

            public Sensor<N2, N1, N2> newPosition() {
                return new PositionSensor() {
                    // public Matrix<N1, N1> stdev() {
                    // return VecBuilder.fill(0.1);
                    // }
                };
            }

            public Sensor<N2, N1, N2> newVelocity() {
                return new VelocitySensor() {
                    // public Matrix<N1, N1> stdev() {
                    // return VecBuilder.fill(0.00001);
                    // }
                };
            }
        };
        IntegratingPredictor<N2, N1> predictor = new IntegratingPredictor<>();
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>();
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, predictor, pointEstimator, pooling);

        // start in negative territory
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);

        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);

        // check that the velocity measurement is right.
        // it has high variance for the don't-know row and low variance for the
        // measurement
        RandomVector<N2> yvel = yvel(-0.240);
        assertArrayEquals(new double[] { 0, -0.240, }, yvel.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1e9, 0, 0, 0.1 }, yvel.P.getData(), kDelta);

        // the measurement and xhat have the same variance so the result should be in
        // the middle
        xhat = estimator.correct(xhat, yvel, system.velocity());
        assertArrayEquals(new double[] { -3.132, -0.12 }, xhat.x.getData(), kDelta);

        xhat = estimator.correct(xhat, yvel(-0.480), system.velocity());
        assertArrayEquals(new double[] { -3.132, -0.312 }, xhat.x.getData(), kDelta);

        xhat = estimator.correct(xhat, yvel(-0.720), system.velocity());
        assertArrayEquals(new double[] { -3.132, -0.55 }, xhat.x.getData(), kDelta);
    }

    @Test
    public void testObserverWrappingCorrectPositionOnly() {
        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D() {
            public Sensor<N2, N1, N2> newFull() {
                return new FullSensor() {
                    // public Matrix<N2, N1> stdev() {
                    // return VecBuilder.fill(0.00001, 0.1);
                    // }
                };
            }

            public Sensor<N2, N1, N2> newPosition() {
                return new PositionSensor() {
                    // public Matrix<N1, N1> stdev() {
                    // return VecBuilder.fill(0.00001);
                    // }
                    // if you want angle modulus, do it here.
                    ////////////////// ?? or not?
                    //
                    //
                    //
                    public AngularRandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
                        AngularRandomVector<N2> x = super.hinv(y, u);
                        // wrap the angle here
                        x.x.set(0, 0, MathUtil.angleModulus(x.x.get(0, 0)));
                        return x;
                    }
                };
            }

            public Sensor<N2, N1, N2> newVelocity() {
                return new VelocitySensor() {
                    // public Matrix<N1, N1> stdev() {
                    // return VecBuilder.fill(0.1);
                    // }
                };
            }
        };
        IntegratingPredictor<N2, N1> predictor = new IntegratingPredictor<>();
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>();
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, predictor, pointEstimator, pooling);

        // start in negative territory, a little positive of -PI.
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);

        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);

        xhat = estimator.correct(xhat, ypos(-3.3), system.position());
        assertArrayEquals(new double[] { 3.067, 0 }, xhat.x.getData(), kDelta);

        xhat = estimator.correct(xhat, ypos(-3.5), system.position());
        assertArrayEquals(new double[] { 2.920, 0 }, xhat.x.getData(), kDelta);

    }

    @Test
    public void testObserverWrappingPredictAndCorrect() {
        // just test the observer across the boundary
        // with both predict and correct
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1> predictor = new IntegratingPredictor<>();
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>();
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, predictor, pointEstimator, pooling);

        // initially, state estimate: near -pi, motionless
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);

        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // saturate negative-going
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, ypos(-3.134), system.position());
        assertArrayEquals(new double[] { -3.134, -0.240 }, xhat.x.getData(), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, ypos(-3.141), system.position());
        assertArrayEquals(new double[] { -3.141, -0.480, }, xhat.x.getData(), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, ypos(3.13), system.position());
        assertArrayEquals(new double[] { 3.130, -0.720 }, xhat.x.getData(), kDelta);

        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, ypos(3.113), system.position());
        assertArrayEquals(new double[] { 3.113, -0.960 }, xhat.x.getData(), kDelta);

    }

}
