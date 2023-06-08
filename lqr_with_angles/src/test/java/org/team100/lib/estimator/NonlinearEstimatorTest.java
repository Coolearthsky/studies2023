package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.Sensor;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.NormalDoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class NonlinearEstimatorTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    private RandomVector<N1> y1(double yd) {
        return new RandomVector<>(VecBuilder.fill(yd), VecBuilder.fill(0));
    }

    @Test
    public void testObserverWrappingPredictOnly() {
        // just test the observer prediction across the boundary
        // it just predicts over and over.
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);

        // initially, state estimate: at zero, motionless
        RandomVector<N2> xhat = new RandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0),
                new Matrix<>(Nat.N2(), Nat.N2()));

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
                    public Matrix<N2, N1> stdev() {
                        return VecBuilder.fill(0.1, 0.00001);
                    }
                };
            }

            public Sensor<N2, N1, N1> newPosition() {
                return new PositionSensor() {
                    public Matrix<N1, N1> stdev() {
                        return VecBuilder.fill(0.1);
                    }
                };
            }

            public Sensor<N2, N1, N1> newVelocity() {
                return new VelocitySensor() {
                    public Matrix<N1, N1> stdev() {
                        return VecBuilder.fill(0.00001);
                    }
                };
            }
        };
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);

        // start in negative territory
        RandomVector<N2> xhat = new RandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0),
                new Matrix<>(Nat.N2(), Nat.N2()));
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(-0.240), system.velocity());
        assertEquals(-3.134, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.12, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(-0.480), system.velocity());
        assertEquals(-3.137, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.3, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(-0.720), system.velocity());
        assertEquals(3.141, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.51, xhat.x.get(1, 0), kDelta);
    }

    @Test
    public void testObserverWrappingCorrectPositionOnly() {
        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D() {
            public Sensor<N2, N1, N2> newFull() {
                return new FullSensor() {
                    public Matrix<N2, N1> stdev() {
                        return VecBuilder.fill(0.00001, 0.1);
                    }
                };
            }

            public Sensor<N2, N1, N1> newPosition() {
                return new PositionSensor() {
                    public Matrix<N1, N1> stdev() {
                        return VecBuilder.fill(0.00001);
                    }
                };
            }

            public Sensor<N2, N1, N1> newVelocity() {
                return new VelocitySensor() {
                    public Matrix<N1, N1> stdev() {
                        return VecBuilder.fill(0.1);
                    }
                };
            }
        };
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);

        // start in negative territory
        RandomVector<N2> xhat = new RandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0),
                new Matrix<>(Nat.N2(), Nat.N2()));
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // supply unwrapped corrections
        xhat = estimator.correct(xhat, y1(-3.3), system.position());
        // filter wraps it
        assertEquals(3.067, xhat.x.get(0, 0), kDelta);
        assertEquals(-0.760, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(-3.5), system.position());
        assertEquals(2.925, xhat.x.get(0, 0), kDelta);
        assertEquals(-2.044, xhat.x.get(1, 0), kDelta);
    }
    @Test
    public void testObserverWrappingPredictAndCorrect() {
        // just test the observer across the boundary
        // with both predict and correct
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);

        // initially, state estimate: near -pi, motionless
        RandomVector<N2> xhat = new RandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0),
                new Matrix<>(Nat.N2(), Nat.N2()));

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
}
