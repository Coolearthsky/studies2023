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

    private AngularRandomVector<N1> y1(double yd) {
        return new AngularRandomVector<>(VecBuilder.fill(yd), VecBuilder.fill(0.1));
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
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, predictor, pooling);

        // initially, state estimate: at zero, motionless
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);

        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // saturate negative-going
        // final double u = -12;
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        // update 1
        xhat = estimator.predictState(xhat, u, kDt);
        assertEquals(-3.134, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.x.get(1, 0), kDelta);

        // update 2
        xhat = estimator.predictState(xhat, u, kDt);
        assertEquals(-3.141, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.x.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: now it wraps around :-)
        // this only works with my wrapping override for predict().
        xhat = estimator.predictState(xhat, u, kDt);
        assertEquals(3.130, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.x.get(1, 0), kDelta);

        // update 4:
        xhat = estimator.predictState(xhat, u, kDt);
        assertEquals(3.113, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.x.get(1, 0), kDelta);
    }

    @Test
    public void testObserverWrappingCorrectVelocityOnly() {
        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D() {
            public Sensor<N2, N1, N2> newFull() {
                return new FullSensor() {
                    // public Matrix<N2, N1> stdev() {
                    //     return VecBuilder.fill(0.1, 0.00001);
                    // }
                };
            }

            public Sensor<N2, N1, N1> newPosition() {
                return new PositionSensor() {
                    // public Matrix<N1, N1> stdev() {
                    //     return VecBuilder.fill(0.1);
                    // }
                };
            }

            public Sensor<N2, N1, N1> newVelocity() {
                return new VelocitySensor() {
                    // public Matrix<N1, N1> stdev() {
                    //     return VecBuilder.fill(0.00001);
                    // }
                };
            }
        };
        IntegratingPredictor<N2, N1> predictor = new IntegratingPredictor<>();
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, predictor, pooling);

        // start in negative territory
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(-0.240), system.velocity());
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        // assertEquals(-3.134, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.12, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(-0.480), system.velocity());
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        // assertEquals(-3.137, xhat.x.get(0, 0), kDelta);
        // assertEquals(-0.3, xhat.x.get(1, 0), kDelta);
        assertEquals(-0.312, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(-0.720), system.velocity());
        // assertEquals(3.141, xhat.x.get(0, 0), kDelta);
        // we never correct position so it never changes
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        // assertEquals(-0.51, xhat.x.get(1, 0), kDelta);
        assertEquals(-0.55, xhat.x.get(1, 0), kDelta);
    }

    @Test
    public void testObserverWrappingCorrectPositionOnly() {
        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D() {
            public Sensor<N2, N1, N2> newFull() {
                return new FullSensor() {
                    // public Matrix<N2, N1> stdev() {
                    //     return VecBuilder.fill(0.00001, 0.1);
                    // }
                };
            }

            public Sensor<N2, N1, N1> newPosition() {
                return new PositionSensor() {
                    // public Matrix<N1, N1> stdev() {
                    //     return VecBuilder.fill(0.00001);
                    // }
                    // if you want angle modulus, do it here.
                    //////////////////?? or not?
                    //
                    //
                    //
                    public AngularRandomVector<N2> hinv(RandomVector<N1> y, Matrix<N1, N1> u) {
                        AngularRandomVector<N2> x = super.hinv(y,u);
                        // wrap the angle here
                        x.x.set(0,0,MathUtil.angleModulus(x.x.get(0,0)));
                        return x;
                    }
                };
            }

            public Sensor<N2, N1, N1> newVelocity() {
                return new VelocitySensor() {
                    // public Matrix<N1, N1> stdev() {
                    //     return VecBuilder.fill(0.1);
                    // }
                };
            }
        };
        IntegratingPredictor<N2, N1> predictor = new IntegratingPredictor<>();
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, predictor, pooling);

        // start in negative territory, a little positive of -PI.
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // supply unwrapped corrections
        // is that ok?  who is responsible for wrapping?
        //

        // so this is saying -3.3 which is really saying +2.98
        xhat = estimator.correct(xhat, y1(-3.3), system.position());
        // filter wraps it .. not yet?
        //??
        // assertEquals(3.067, xhat.x.get(0, 0), kDelta);

        // this is beyond pi
        //???

//        assertEquals(-3, xhat.x.get(0, 0), kDelta);
        assertEquals(3.067, xhat.x.get(0, 0), kDelta);
        // assertEquals(-0.760, xhat.x.get(1, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(-3.5), system.position());
        assertEquals(2.920, xhat.x.get(0, 0), kDelta);
        // we never correct velocity so it never changes
        assertEquals(0, xhat.x.get(1, 0), kDelta);
//        assertEquals(-2.044, xhat.x.get(1, 0), kDelta);
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
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, predictor, pooling);

        // initially, state estimate: near -pi, motionless
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);

        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // saturate negative-going
        // final double u = -12;
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        // update 1
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, y1(-3.134), system.position());

        assertEquals(-3.134, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.x.get(1, 0), kDelta);

        // update 2
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, y1(-3.141), system.position());

        assertEquals(-3.141, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.x.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: now it wraps around :-)
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, y1(3.13), system.position());

        assertEquals(3.130, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.x.get(1, 0), kDelta);

        // update 4:
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, y1(3.113), system.position());

        assertEquals(3.113, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.x.get(1, 0), kDelta);
    }


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

    @Test
    public void testStateForMeasurement() {

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1> predictor = new IntegratingPredictor<>();
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, predictor, pooling);
        // initial xhat is zero
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
        xP.set(0, 0, 0.01);
        xP.set(1, 1, 0.01);
        RandomVector<N2> xhat = new RandomVector<>(xx, xP);

        // measurement is 1,0
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.01);
        yP.set(1, 1, 0.01);
        RandomVector<N2> y = new RandomVector<>(yx, yP);

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        xhat = observer.stateForMeasurement(u, y, this::hinv);
        // since the state is just the measurement,
        // you get the specified mean and variance of the measurement.
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.01, 0, 0, 0.01 }, xhat.P.getData(), kDelta);
    }

    /**
     * this method converges *much* faster than the "kalman gain" method, and it
     * seems more correct and less mysterious.
     */
    @Test
    public void testCombineStateForMeasurement() {
        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1> predictor = new IntegratingPredictor<>();
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, predictor, pooling);

        // initial xhat is zero
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
        xP.set(0, 0, 0.01);
        xP.set(1, 1, 0.01);
        RandomVector<N2> xhat = new RandomVector<>(xx, xP);
        RandomVector<N2> xhatNew = xhat.copy();

        // measurement is 1,0
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.01);
        yP.set(1, 1, 0.01);
        RandomVector<N2> y = new RandomVector<>(yx, yP);

        LinearPooling<N2> p2 = new VarianceWeightedLinearPooling<N2>();

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        xhatNew = observer.stateForMeasurement(u, y, this::hinv);
        xhat = p2.fuse(xhat, xhatNew);
        // since the old and new have the same variance the mean is in the middle
        assertArrayEquals(new double[] { 0.5, 0 }, xhat.x.getData(), kDelta);
        // the difference in means adds to the variance but only of the first component
        assertArrayEquals(new double[] { 0.26, 0, 0, 0.01 }, xhat.P.getData(), kDelta);

        xhatNew = observer.stateForMeasurement(u, y, this::hinv);
        xhat = p2.fuse(xhat, xhatNew);
        // new measurement has lower variance so it is preferred
        assertArrayEquals(new double[] { 0.982, 0 }, xhat.x.getData(), kDelta);
        // mean dispersion keeps increasing P
        assertArrayEquals(new double[] { 0.028, 0, 0, 0.01 }, xhat.P.getData(), kDelta);

        xhatNew = observer.stateForMeasurement(u, y, this::hinv);
        xhat = p2.fuse(xhat, xhatNew);
        assertArrayEquals(new double[] { 0.995, 0 }, xhat.x.getData(), kDelta);
        // mean dispersion is on the way down now
        assertArrayEquals(new double[] { 0.015, 0, 0, 0.01 }, xhat.P.getData(), kDelta);

        // go all the way to the end
        for (int i = 0; i < 100; ++i) {
            xhatNew = observer.stateForMeasurement(u, y, this::hinv);
            xhat = p2.fuse(xhat, xhatNew);
        }
        // now the estimate matches the measurement
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.01, 0, 0, 0.01 }, xhat.P.getData(), kDelta);
    }

}
