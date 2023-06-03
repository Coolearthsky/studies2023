package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.LinearizedLQR;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Demonstrates ExtendedKalmanFilter in angle-wrapping scenarios, and also a
 * custom LQR that implements wrapping as well.
 */
public class EKFTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    final Vector<N2> stateTolerance = VecBuilder
            .fill(0.01, // angle (rad)
                    0.2); // velocity (rad/s)
    final Vector<N1> controlTolerance = VecBuilder
            .fill(12.0); // output (volts)

    @Test
    public void testObserver() {
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(0.01, 0.1, 0.015, 0.17);
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, kDt);
        // obsP: error covariance
        assertEquals(0.00064, observer.getP(0, 0), 0.0001);
        assertEquals(0.0015, observer.getP(1, 0), 0.0001);
    }

    @Test
    public void testNearZero() {
        // positive setpoint, delta +0.02, push positive

        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(0.01, 0.1, 0.015, 0.17);
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, kDt);
        LinearizedLQR<N2, N1, N2> controller = new LinearizedLQR<>(system, stateTolerance, controlTolerance);

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(0, xhat.get(0, 0));
        assertEquals(0, xhat.get(1, 0));

        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);

        // initial: push hard to get started
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(11.455, u.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.002), system.position());
        xhat = observer.getXhat();
        assertEquals(0.002, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.008, u.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.006), system.position());
        xhat = observer.getXhat();
        assertEquals(0.006, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-2.558, u.get(0, 0), kDelta);

        // update 3: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.01), system.position());
        xhat = observer.getXhat();
        assertEquals(0.010, xhat.get(0, 0), kDelta);
        assertEquals(0.177, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-2.556, u.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.013), system.position());
        xhat = observer.getXhat();
        assertEquals(0.013, xhat.get(0, 0), kDelta);
        assertEquals(0.126, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.971, u.get(0, 0), kDelta);

        // update 5: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.015), system.position());
        xhat = observer.getXhat();
        assertEquals(0.015, xhat.get(0, 0), kDelta);
        assertEquals(0.086, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.38, u.get(0, 0), kDelta);

        // update 6: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.017), system.position());
        xhat = observer.getXhat();
        assertEquals(0.017, xhat.get(0, 0), kDelta);
        assertEquals(0.059, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.976, u.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.018), system.position());
        xhat = observer.getXhat();
        assertEquals(0.018, xhat.get(0, 0), kDelta);
        assertEquals(0.039, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.663, u.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.019), system.position());
        xhat = observer.getXhat();
        assertEquals(0.019, xhat.get(0, 0), kDelta);
        assertEquals(0.026, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.467, u.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.02), system.position());
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.017, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.359, u.get(0, 0), kDelta);

        // update 10: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.02), system.position());
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.01, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.228, u.get(0, 0), kDelta);

        // update 11: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.02), system.position());
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.005, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.131, u.get(0, 0), kDelta);

        // update 12: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.02), system.position());
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.071, u.get(0, 0), kDelta);

        // update 13: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.02), system.position());
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.001, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.037, u.get(0, 0), kDelta);

        // update 14: pretty much done
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(0.02), system.position());
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.019, u.get(0, 0), kDelta);
    }

    @Test
    public void testNearPiWithoutWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is pi - 0.03
        // so delta is +0.02, should push positive

        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(0.01, 0.1, 0.015, 0.17);
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, kDt);
        LinearizedLQR<N2, N1, N2> controller = new LinearizedLQR<>(system, stateTolerance, controlTolerance);

        observer.setXhat(VecBuilder.fill(Math.PI - 0.03, 0));

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(3.112, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        observer.correct(VecBuilder.fill(Math.PI - 0.03), system.position());
        xhat = observer.getXhat();
        assertEquals(3.112, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint, kDt);
        // should be same output as above since the motion is the same
        assertEquals(11.455, u.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.114), system.position());
        xhat = observer.getXhat();
        assertEquals(3.114, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.024, u.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.118), system.position());
        xhat = observer.getXhat();
        assertEquals(3.118, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-2.591, u.get(0, 0), kDelta);

        // update 3: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.122), system.position());
        xhat = observer.getXhat();
        assertEquals(3.122, xhat.get(0, 0), kDelta);
        assertEquals(0.177, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-2.58, u.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.125), system.position());
        xhat = observer.getXhat();
        assertEquals(3.125, xhat.get(0, 0), kDelta);
        assertEquals(0.125, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.987, u.get(0, 0), kDelta);

        // update 5: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.128), system.position());
        xhat = observer.getXhat();
        assertEquals(3.128, xhat.get(0, 0), kDelta);
        assertEquals(0.086, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.465, u.get(0, 0), kDelta);

        // update 6: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.13), system.position());
        xhat = observer.getXhat();
        assertEquals(3.13, xhat.get(0, 0), kDelta);
        assertEquals(0.056, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.052, u.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.131), system.position());
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.035, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.719, u.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.131), system.position());
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.021, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.431, u.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.131), system.position());
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.012, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.240, u.get(0, 0), kDelta);

        // update 10: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.131), system.position());
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.007, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.126, u.get(0, 0), kDelta);

        // update 11: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.131), system.position());
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.005, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.062, u.get(0, 0), kDelta);

        // update 12: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.131), system.position());
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.028, u.get(0, 0), kDelta);

        // update 13: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.131), system.position());
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.01, u.get(0, 0), kDelta);

        // update 14: pretty much done
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.131), system.position());
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.001, u.get(0, 0), kDelta);
    }

    @Test
    public void testNearPiWithWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(0.01, 0.1, 0.015, 0.17);
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, kDt);
        LinearizedLQR<N2, N1, N2> controller = new LinearizedLQR<>(system, stateTolerance, controlTolerance);

        // starting point is the only difference
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // starting point is the only difference
        observer.correct(VecBuilder.fill(-1.0 * Math.PI + 0.01), system.position());
        xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint, kDt);
        // using the EKF this is the correct negative number
        assertEquals(-11.455, u.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(-3.133), system.position());
        xhat = observer.getXhat();
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-3.133, xhat.get(0, 0), kDelta);
        assertEquals(-0.229, xhat.get(1, 0), kDelta);
        assertEquals(-0.048, u.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(-3.138), system.position());
        xhat = observer.getXhat();
        assertEquals(-3.138, xhat.get(0, 0), kDelta);
        assertEquals(-0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(2.594, u.get(0, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: still slowing down
        // note boundary crossing here
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.141), system.position());
        xhat = observer.getXhat();
        assertEquals(3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.177, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(2.612, u.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.138), system.position());
        xhat = observer.getXhat();
        assertEquals(3.138, xhat.get(0, 0), kDelta);
        assertEquals(-0.125, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(2.016, u.get(0, 0), kDelta);

        // update 5: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.135), system.position());
        xhat = observer.getXhat();
        assertEquals(3.135, xhat.get(0, 0), kDelta);
        assertEquals(-0.086, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(1.485, u.get(0, 0), kDelta);

        // update 6: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.134), system.position());
        xhat = observer.getXhat();
        assertEquals(3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.056, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.991, u.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.133), system.position());
        xhat = observer.getXhat();
        assertEquals(3.133, xhat.get(0, 0), kDelta);
        assertEquals(-0.036, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.656, u.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.132), system.position());
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.023, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.460, u.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.132), system.position());
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.014, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.279, u.get(0, 0), kDelta);

        // update 10: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.132), system.position());
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.008, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.155, u.get(0, 0), kDelta);

        // update 11: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.132), system.position());
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.005, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.082, u.get(0, 0), kDelta);

        // update 12: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.132), system.position());
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.040, u.get(0, 0), kDelta);

        // update 13: almost there
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.132), system.position());
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.018, u.get(0, 0), kDelta);

        // update 14: pretty much done
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.132), system.position());
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.006, u.get(0, 0), kDelta);
    }

    @Test
    public void testObserverWrappingPredictOnly() {
        // just test the observer prediction across the boundary
        // it just predicts over and over.
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(0.01, 0.1, 0.015, 0.17);
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, kDt);

        // initially, state estimate: at zero, motionless
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // saturate negative-going
        // final double u = -12;
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        // update 1
        observer.predictState(u, kDt);
        xhat = observer.getXhat();
        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.get(1, 0), kDelta);

        // update 2
        observer.predictState(u, kDt);
        xhat = observer.getXhat();
        assertEquals(-3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: now it wraps around :-)
        // this only works with my wrapping override for predict().
        observer.predictState(u, kDt);
        xhat = observer.getXhat();
        assertEquals(3.130, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        // update 4:
        observer.predictState(u, kDt);
        xhat = observer.getXhat();
        assertEquals(3.113, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);
    }

    @Test
    public void testObserverWrappingCorrectVelocityOnly() {
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(0.1, 0.00001, 0.015, 0.17);
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, kDt);

        // start in negative territory
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertAll(
                () -> assertEquals(-3.132, observer.getXhat(0), kDelta),
                () -> assertEquals(0, observer.getXhat(1), kDelta));

        // note that velocity corrections don't affect position
        // because we're never predicting i.e. time does not pass.
        observer.correct(VecBuilder.fill(-0.240), system.velocity());
        assertAll(
                () -> assertEquals(-3.134, observer.getXhat(0), kDelta),
                () -> assertEquals(-0.239, observer.getXhat(1), kDelta));

        observer.correct(VecBuilder.fill(-0.480), system.velocity());
        assertAll(
                () -> assertEquals(-3.135, observer.getXhat(0), kDelta),
                () -> assertEquals(-0.360, observer.getXhat(1), kDelta));

        observer.correct(VecBuilder.fill(-0.720), system.velocity());
        assertAll(
                () -> assertEquals(-3.136, observer.getXhat(0), kDelta),
                () -> assertEquals(-0.480, observer.getXhat(1), kDelta));
    }

    @Test
    public void testObserverWrappingCorrectPositionOnly() {
        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(0.00001, 0.1, 0.015, 0.17);
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, kDt);

        // start in negative territory
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertAll(
                () -> assertEquals(-3.132, observer.getXhat(0), kDelta),
                () -> assertEquals(0, observer.getXhat(1), kDelta));

        // supply unwrapped corrections
        observer.correct(VecBuilder.fill(-3.3), system.position());
        // filter wraps it
        assertAll(
                () -> assertEquals(2.983, observer.getXhat(0), kDelta),
                () -> assertEquals(-1.692, observer.getXhat(1), kDelta));

        observer.correct(VecBuilder.fill(-3.5), system.position());
        assertAll(
                () -> assertEquals(2.883, observer.getXhat(0), kDelta),
                () -> assertEquals(-2.698, observer.getXhat(1), kDelta));
    }

    @Test
    public void testObserverWrappingPredictAndCorrect() {
        // just test the observer across the boundary
        // with both predict and correct
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(0.01, 0.1, 0.015, 0.17);
        NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, kDt);

        // initially, state estimate: near -pi, motionless
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // saturate negative-going
        // final double u = -12;
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        // update 1
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(-3.134), system.position());
        xhat = observer.getXhat();
        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.get(1, 0), kDelta);

        // update 2
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(-3.141), system.position());
        xhat = observer.getXhat();
        assertEquals(-3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: now it wraps around :-)
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.13), system.position());
        xhat = observer.getXhat();
        assertEquals(3.130, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        // update 4:
        observer.predictState(u, kDt);
        observer.correct(VecBuilder.fill(3.113), system.position());
        xhat = observer.getXhat();
        assertEquals(3.113, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);
    }

}
