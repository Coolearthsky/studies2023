package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Map.Entry;

import org.junit.jupiter.api.Test;
import org.team100.lib.storage.BitemporalBuffer;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.CartesianPlant1D;
import org.team100.lib.system.examples.NormalDoubleIntegratorCartesian1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class BitemporalEstimatorTest {
    private static final double kDelta = 0.001;

    @Test
    public void testSimple() {
        NonlinearPlant<N2, N1, N2> plant = new NormalDoubleIntegratorCartesian1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(plant, 0.01);
        BitemporalBuffer<Matrix<N2, N1>> stateBuffer = new BitemporalBuffer<>(1000);
        // no motion
        stateBuffer.put(0l, 0.0, VecBuilder.fill(0, 0));
        BitemporalEstimator<N2, N1, N2> bitemporalEstimator = new BitemporalEstimator<>(plant, stateBuffer, estimator);
        {
            // this should find the initial state
            Matrix<N2, N1> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 0, 0);
            assertAll(
                    () -> assertEquals(0, prediction.get(0, 0), kDelta),
                    () -> assertEquals(0, prediction.get(1, 0), kDelta));
        }
        {
            // since there is no motion this should be the same
            Matrix<N2, N1> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 10, 1.0);
            assertAll(
                    () -> assertEquals(0, prediction.get(0, 0), kDelta),
                    () -> assertEquals(0, prediction.get(1, 0), kDelta));
        }
    }

    @Test
    public void testMotion() {
        NonlinearPlant<N2, N1, N2> plant = new NormalDoubleIntegratorCartesian1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(plant, 0.01);
        BitemporalBuffer<Matrix<N2, N1>> stateBuffer = new BitemporalBuffer<>(1000);
        // velocity = 1
        stateBuffer.put(0l, 0.0, VecBuilder.fill(0, 1));
        BitemporalEstimator<N2, N1, N2> bitemporalEstimator = new BitemporalEstimator<>(plant, stateBuffer, estimator);
        {
            // this should find the initial state
            Matrix<N2, N1> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 0, 0);
            assertAll(
                    () -> assertEquals(0, prediction.get(0, 0), kDelta),
                    () -> assertEquals(1, prediction.get(1, 0), kDelta));
        }
        {
            // with v=1 we should be at p=1 after 1 sec
            Matrix<N2, N1> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 10, 1.0);
            assertAll(
                    () -> assertEquals(1, prediction.get(0, 0), kDelta),
                    () -> assertEquals(1, prediction.get(1, 0), kDelta));
        }
        {
            // extrapolate really far, should be at p=100
            Matrix<N2, N1> prediction = bitemporalEstimator.predict(VecBuilder.fill(0), 10, 100.0);
            assertAll(
                    () -> assertEquals(100, prediction.get(0, 0), kDelta),
                    () -> assertEquals(1, prediction.get(1, 0), kDelta));
        }
    }

    /** correct position and velocity separately every 0.005s */
    @Test
    public void testCorrection() {
        // System.out.println("SEPARATE");
        CartesianPlant1D plant = new NormalDoubleIntegratorCartesian1D();
        // TODO: the constant timestep here will not work.
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(plant, 0.01);
        BitemporalBuffer<Matrix<N2, N1>> stateBuffer = new BitemporalBuffer<>(1000);
        long recordTime = 0l;
        double validTime = 0;
        stateBuffer.put(recordTime, validTime, VecBuilder.fill(0, 0));
        BitemporalEstimator<N2, N1, N2> bitemporalEstimator = new BitemporalEstimator<>(plant, stateBuffer, estimator);
        Matrix<N2, N1> xhat;// = estimator.getXhat();
        // Matrix<N2, N2> P = estimator.getP();
        // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
        // validTime, xhat.get(0, 0),
        // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));

        for (long i = 10; i < 1000; i += 10) {
            recordTime = i;
            validTime = 0.001 * i;
            xhat = bitemporalEstimator.correct(VecBuilder.fill(1), plant.position(), recordTime, validTime);
            // P = estimator.getP();
            // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
            // validTime, xhat.get(0, 0),
            // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));

            recordTime += 5;
            validTime += 0.005;
            xhat = bitemporalEstimator.correct(VecBuilder.fill(0), plant.velocity(), recordTime, validTime);
            // P = estimator.getP();
            // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
            // validTime, xhat.get(0, 0),
            // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));
        }

        // final correction
        xhat = bitemporalEstimator.correct(VecBuilder.fill(1), plant.position(), 1000l, 1.0);
        xhat = bitemporalEstimator.correct(VecBuilder.fill(0), plant.velocity(), 1010l, 1.005);
        // these are now the same because the time-step is the same fixed number as
        // below (0.01 s)
        // these are the EKF values
        // assertEquals(0.814, xhat.get(0, 0), kDelta);
        //assertEquals(1.435, xhat.get(1, 0), kDelta);
        // these are the new values; a little different because of constant (a little higher) gain.
        assertEquals(0.945, xhat.get(0, 0), kDelta);
        assertEquals(1.655, xhat.get(1, 0), kDelta);
    }

    /**
     * correct position and velocity together every 0.01s.
     * the combined update works faster because of the dt scaling.
     * there's a copy of this test in EKFTest to verify the behavior is the same.
     */
    @Test
    public void testFullCorrection() {
        // System.out.println("FULL");
        CartesianPlant1D plant = new NormalDoubleIntegratorCartesian1D();
        // TODO: the constant timestep here will not work.
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(plant, 0.01);
        BitemporalBuffer<Matrix<N2, N1>> stateBuffer = new BitemporalBuffer<>(1000);
        long recordTime = 0l;
        double validTime = 0;
        stateBuffer.put(recordTime, validTime, VecBuilder.fill(0, 0));
        BitemporalEstimator<N2, N1, N2> bitemporalEstimator = new BitemporalEstimator<>(plant, stateBuffer, estimator);
        Matrix<N2, N1> xhat;// = estimator.getXhat();
        // Matrix<N2, N2> P = estimator.getP();
        // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
        // validTime, xhat.get(0, 0),
        // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));

        for (long i = 10; i < 1000; i += 10) {
            recordTime = i;
            validTime = 0.001 * i;
            xhat = bitemporalEstimator.correct(VecBuilder.fill(1, 0), plant.full(), recordTime, validTime);
            // P = estimator.getP();
            // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
            // validTime, xhat.get(0, 0),
            // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));
        }

        // final correction
        xhat = bitemporalEstimator.correct(VecBuilder.fill(1, 0), plant.full(), 1000l, 1.0);
        // these are identical to the real EKF, using fixed time-step 0.01 s
        //assertEquals(0.814, xhat.get(0, 0), kDelta);
        //assertEquals(1.435, xhat.get(1, 0), kDelta);
        // these are the new ones, a little higher because of the higher constant gain.
        assertEquals(0.948, xhat.get(0, 0), kDelta);
        assertEquals(1.659, xhat.get(1, 0), kDelta);

    }

    @Test
    public void testFloor() {
        CartesianPlant1D plant = new NormalDoubleIntegratorCartesian1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(plant, 0.01);
        BitemporalBuffer<Matrix<N2, N1>> stateBuffer = new BitemporalBuffer<>(1000);
        BitemporalEstimator<N2, N1, N2> bitemporalEstimator = new BitemporalEstimator<>(plant, stateBuffer, estimator);
        {
            // uninitialized
            Throwable exception = assertThrows(IllegalStateException.class, () -> bitemporalEstimator.floor(1));
            assertEquals("No floor key (not initialized?): 1.0", exception.getMessage());
        }

        stateBuffer.put(0l, 0.0, VecBuilder.fill(0, 0));
        {
            Entry<Double, Entry<Long, Matrix<N2, N1>>> floor = bitemporalEstimator.floor(0);
            Matrix<N2, N1> x = floor.getValue().getValue();
            double t = floor.getKey();
            assertAll(
                    () -> assertEquals(0, x.get(0, 0), kDelta),
                    () -> assertEquals(0, x.get(1, 0), kDelta),
                    () -> assertEquals(0, t, kDelta));
        }

        {
            Throwable exception = assertThrows(IllegalArgumentException.class, () -> bitemporalEstimator.floor(-1));
            assertEquals("Negative time is not allowed: -1.0", exception.getMessage());
        }
        {
            // just one state in the buffer
            Entry<Double, Entry<Long, Matrix<N2, N1>>> floor = bitemporalEstimator.floor(10);
            Matrix<N2, N1> x = floor.getValue().getValue();
            double t = floor.getKey();
            assertAll(
                    () -> assertEquals(0, x.get(0, 0), kDelta),
                    () -> assertEquals(0, x.get(1, 0), kDelta),
                    () -> assertEquals(0, t, kDelta));
        }

    }

}
