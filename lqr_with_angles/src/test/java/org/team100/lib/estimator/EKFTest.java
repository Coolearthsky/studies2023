package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.ConstantGainLinearizedLQR;
import org.team100.lib.system.Sensor;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.NormalDoubleIntegratorRotary1D;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.NumericalJacobian;

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
        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);
        // obsP: error covariance
        assertEquals(0.00064, estimator.getP().get(0, 0), 0.0001);
        assertEquals(0.0015, estimator.getP().get(1, 0), 0.0001);
    }

    @Test
    public void testNearZero() {
        // positive setpoint, delta +0.02, push positive

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
                stateTolerance, controlTolerance, kDt);

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = VecBuilder.fill(0, 0);
        assertEquals(0, xhat.get(0, 0));
        assertEquals(0, xhat.get(1, 0));

        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);

        // initial: push hard to get started
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint);
        assertEquals(11.455, u.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.002), system.position());
        assertEquals(0.002, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.005, u.get(0, 0), kDelta);

        // update 2: slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.006), system.position());
        assertEquals(0.006, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-2.564, u.get(0, 0), kDelta);

        // update 3: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.01), system.position());
        assertEquals(0.010, xhat.get(0, 0), kDelta);
        assertEquals(0.177, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-2.561, u.get(0, 0), kDelta);

        // update 4: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.013), system.position());
        assertEquals(0.013, xhat.get(0, 0), kDelta);
        assertEquals(0.126, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-1.975, u.get(0, 0), kDelta);

        // update 5: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.015), system.position());
        assertEquals(0.015, xhat.get(0, 0), kDelta);
        assertEquals(0.086, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-1.383, u.get(0, 0), kDelta);

        // update 6: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.017), system.position());
        assertEquals(0.017, xhat.get(0, 0), kDelta);
        assertEquals(0.059, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.973, u.get(0, 0), kDelta);

        // update 7: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.018), system.position());
        assertEquals(0.018, xhat.get(0, 0), kDelta);
        assertEquals(0.039, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.659, u.get(0, 0), kDelta);

        // update 8: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.019), system.position());
        assertEquals(0.019, xhat.get(0, 0), kDelta);
        assertEquals(0.026, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.462, u.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.02), system.position());
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.017, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.350, u.get(0, 0), kDelta);

        // update 10: almost there
        estimator.predictState(xhat, u, kDt);
        estimator.correct(xhat, VecBuilder.fill(0.02), system.position());
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.01, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.223, u.get(0, 0), kDelta);

        // update 11: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.02), system.position());
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.005, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.131, u.get(0, 0), kDelta);

        // update 12: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.02), system.position());
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.072, u.get(0, 0), kDelta);

        // update 13: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.02), system.position());
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.001, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.039, u.get(0, 0), kDelta);

        // update 14: pretty much done
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(0.02), system.position());
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.02, u.get(0, 0), kDelta);
    }

    @Test
    public void testNearPiWithoutWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is pi - 0.03
        // so delta is +0.02, should push positive

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
                stateTolerance, controlTolerance, kDt);

        Matrix<N2, N1> xhat = VecBuilder.fill(Math.PI - 0.03, 0);

        // initially, state estimate: at zero, motionless
        assertEquals(3.112, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, VecBuilder.fill(Math.PI - 0.03), system.position());
        assertEquals(3.112, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint);
        // should be same output as above since the motion is the same
        assertEquals(11.455, u.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.114), system.position());
        assertEquals(3.114, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.023, u.get(0, 0), kDelta);

        // update 2: slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.118), system.position());
        assertEquals(3.118, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-2.591, u.get(0, 0), kDelta);

        // update 3: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.122), system.position());
        assertEquals(3.122, xhat.get(0, 0), kDelta);
        assertEquals(0.177, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-2.581, u.get(0, 0), kDelta);

        // update 4: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.125), system.position());
        assertEquals(3.125, xhat.get(0, 0), kDelta);
        assertEquals(0.125, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-1.988, u.get(0, 0), kDelta);

        // update 5: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.128), system.position());
        assertEquals(3.128, xhat.get(0, 0), kDelta);
        assertEquals(0.086, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-1.462, u.get(0, 0), kDelta);

        // update 6: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.13), system.position());
        assertEquals(3.13, xhat.get(0, 0), kDelta);
        assertEquals(0.056, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-1.047, u.get(0, 0), kDelta);

        // update 7: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.131), system.position());
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.035, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.714, u.get(0, 0), kDelta);

        // update 8: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.131), system.position());
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.021, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.431, u.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.131), system.position());
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.012, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.242, u.get(0, 0), kDelta);

        // update 10: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.131), system.position());
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.007, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.129, u.get(0, 0), kDelta);

        // update 11: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.131), system.position());
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.005, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.066, u.get(0, 0), kDelta);

        // update 12: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.131), system.position());
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.031, u.get(0, 0), kDelta);

        // update 13: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.131), system.position());
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.013, u.get(0, 0), kDelta);

        // update 14: pretty much done
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.131), system.position());
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(-0.004, u.get(0, 0), kDelta);
    }

    @Test
    public void testNearPiWithWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
                stateTolerance, controlTolerance, kDt);

        // starting point is the only difference
        Matrix<N2, N1> xhat = VecBuilder.fill(-1.0 * Math.PI + 0.01, 0);

        // initially, state estimate: at zero, motionless
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // starting point is the only difference
        xhat = estimator.correct(xhat, VecBuilder.fill(-1.0 * Math.PI + 0.01), system.position());

        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint);
        // using the EKF this is the correct negative number
        assertEquals(-11.455, u.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(-3.133), system.position());

        u = controller.calculate(xhat, setpoint);
        assertEquals(-3.133, xhat.get(0, 0), kDelta);
        assertEquals(-0.229, xhat.get(1, 0), kDelta);
        assertEquals(-0.048, u.get(0, 0), kDelta);

        // update 2: slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(-3.138), system.position());

        assertEquals(-3.138, xhat.get(0, 0), kDelta);
        assertEquals(-0.229, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(2.595, u.get(0, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: still slowing down
        // note boundary crossing here
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.141), system.position());

        assertEquals(3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.177, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(2.612, u.get(0, 0), kDelta);

        // update 4: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.138), system.position());

        assertEquals(3.138, xhat.get(0, 0), kDelta);
        assertEquals(-0.125, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(2.016, u.get(0, 0), kDelta);

        // update 5: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.135), system.position());

        assertEquals(3.135, xhat.get(0, 0), kDelta);
        assertEquals(-0.086, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(1.482, u.get(0, 0), kDelta);

        // update 6: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.134), system.position());

        assertEquals(3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.056, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.989, u.get(0, 0), kDelta);

        // update 7: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.133), system.position());

        assertEquals(3.133, xhat.get(0, 0), kDelta);
        assertEquals(-0.036, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.656, u.get(0, 0), kDelta);

        // update 8: still slowing down
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.132), system.position());

        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.023, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.457, u.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.132), system.position());

        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.014, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.279, u.get(0, 0), kDelta);

        // update 10: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.132), system.position());

        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.008, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.157, u.get(0, 0), kDelta);

        // update 11: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.132), system.position());

        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.005, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.084, u.get(0, 0), kDelta);

        // update 12: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.132), system.position());

        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.043, u.get(0, 0), kDelta);

        // update 13: almost there
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.132), system.position());

        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.02, u.get(0, 0), kDelta);

        // update 14: pretty much done
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.132), system.position());

        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        u = controller.calculate(xhat, setpoint);
        assertEquals(0.008, u.get(0, 0), kDelta);
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
        Matrix<N2, N1> xhat = VecBuilder.fill(-1.0 * Math.PI + 0.01, 0);

        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // saturate negative-going
        // final double u = -12;
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        // update 1
        xhat = estimator.predictState(xhat, u, kDt);
        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.get(1, 0), kDelta);

        // update 2
        xhat = estimator.predictState(xhat, u, kDt);
        assertEquals(-3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: now it wraps around :-)
        // this only works with my wrapping override for predict().
        xhat = estimator.predictState(xhat, u, kDt);
        assertEquals(3.130, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        // update 4:
        xhat = estimator.predictState(xhat, u, kDt);
        assertEquals(3.113, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);
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
        Matrix<N2, N1> xhat = VecBuilder.fill(-1.0 * Math.PI + 0.01, 0);
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, VecBuilder.fill(-0.240), system.velocity());
        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.12, xhat.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, VecBuilder.fill(-0.480), system.velocity());
        assertEquals(-3.137, xhat.get(0, 0), kDelta);
        assertEquals(-0.3, xhat.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, VecBuilder.fill(-0.720), system.velocity());
        assertEquals(3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.51, xhat.get(1, 0), kDelta);
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
        Matrix<N2, N1> xhat = VecBuilder.fill(-1.0 * Math.PI + 0.01, 0);
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // supply unwrapped corrections
        xhat = estimator.correct(xhat, VecBuilder.fill(-3.3), system.position());
        // filter wraps it
        assertEquals(3.067, xhat.get(0, 0), kDelta);
        assertEquals(-0.760, xhat.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, VecBuilder.fill(-3.5), system.position());
        assertEquals(2.925, xhat.get(0, 0), kDelta);
        assertEquals(-2.044, xhat.get(1, 0), kDelta);
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
        Matrix<N2, N1> xhat = VecBuilder.fill(-1.0 * Math.PI + 0.01, 0);

        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // saturate negative-going
        // final double u = -12;
        final Matrix<N1, N1> u = VecBuilder.fill(-12);

        // update 1
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(-3.134), system.position());

        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.get(1, 0), kDelta);

        // update 2
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(-3.141), system.position());

        assertEquals(-3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: now it wraps around :-)
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.13), system.position());

        assertEquals(3.130, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        // update 4:
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, VecBuilder.fill(3.113), system.position());

        assertEquals(3.113, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);
    }

    public Matrix<N2, N1> f(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u;
        return VecBuilder.fill(pdot, vdot);
    }

    public Matrix<N2, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return x;
    }

    /** Same test as in BitemporalEstimator, to see if it's the same. */
    @Test
    public void testFullCorrection() {
        // System.out.println("FULL EKF");
        Matrix<N2, N1> stateStdDevs = VecBuilder.fill(0.015, 0.17);
        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(0.01, 0.1);
        ExtendedKalmanFilter<N2, N1, N2> estimator = new ExtendedKalmanFilter<>(
                Nat.N2(), Nat.N1(), Nat.N2(), this::f, this::h, stateStdDevs, measurementStdDevs, 0.01);

        // double validTime = 0;
        Matrix<N2, N1> xhat = estimator.getXhat();
        // Matrix<N2, N2> P = estimator.getP();
        // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
        // validTime, xhat.get(0, 0),
        // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));

        for (long i = 10; i < 1000; i += 10) {
            // validTime = 0.001 * i;
            estimator.correct(VecBuilder.fill(0), VecBuilder.fill(1, 0));
            xhat = estimator.getXhat();
            // P = estimator.getP();
            // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
            // validTime, xhat.get(0, 0),
            // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));
        }

        estimator.correct(VecBuilder.fill(0), VecBuilder.fill(1, 0));
        xhat = estimator.getXhat();
        assertEquals(0.814, xhat.get(0, 0), kDelta);
        assertEquals(1.435, xhat.get(1, 0), kDelta);
    }

    /** trying to figure out how to handle the discretization time */
    // turn this off because it makes a lot of output.
    // @Test
    public void testP() {
        printStuff(0.01);
        printStuff(0.1);
    }

    private void printStuff(double dtSeconds) {
        System.out.println("=======================================");
        System.out.println("dt " + dtSeconds);
        Matrix<N2, N1> stateStdDevs = VecBuilder.fill(0.015, 0.17);
        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(0.1, 0.1);

        // this is from the EKF ctor
        Matrix<N2, N2> m_contQ = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stateStdDevs);
        Matrix<N2, N2> m_contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), measurementStdDevs);
        Matrix<N2, N1> m_xHat = new Matrix<>(Nat.N2(), Nat.N1());

        Matrix<N2, N2> contA = NumericalJacobian.numericalJacobianX(
                Nat.N2(), Nat.N2(), this::f, m_xHat, new Matrix<>(Nat.N1(), Nat.N1()));
        System.out.println("contA " + contA);
        // double integrator A
        assertArrayEquals(new double[] { 0, 1, 0, 0 }, contA.getData());

        Matrix<N2, N2> C = NumericalJacobian.numericalJacobianX(
                Nat.N2(), Nat.N2(), this::h, m_xHat, new Matrix<>(Nat.N1(), Nat.N1()));
        System.out.println("C " + C);
        // C is identity since h is identity.
        assertArrayEquals(new double[] { 1, 0, 0, 1 }, C.getData());

        Pair<Matrix<N2, N2>, Matrix<N2, N2>> discPair = Discretization.discretizeAQTaylor(contA, m_contQ, dtSeconds);
        Matrix<N2, N2> discA = discPair.getFirst();
        System.out.println("discA " + discA);
        Matrix<N2, N2> discQ = discPair.getSecond();
        System.out.println("discQ " + discQ);
        Matrix<N2, N2> discR = Discretization.discretizeR(m_contR, dtSeconds);
        System.out.println("discR " + discR);
        Matrix<N2, N2> m_P = Drake.discreteAlgebraicRiccatiEquation(discA.transpose(), C.transpose(), discQ, discR);
        System.out.println("m_P " + m_P);

        // this is from EKF correct()

        // state prediction covariance
        Matrix<N2, N2> S = C.times(m_P).times(C.transpose()).plus(discR);
        System.out.println("S " + S);
        Matrix<N2, N2> K = S.transpose().solve(C.times(m_P.transpose())).transpose();
        System.out.println("K " + K);

        // output
        Matrix<N2, N1> y = VecBuilder.fill(1, 0);
        // input
        Matrix<N1, N1> u = VecBuilder.fill(0);

        Matrix<N2, N1> expectedMeasurement = h(m_xHat, u);
        System.out.println("expectedMeasurement " + expectedMeasurement);
        assertArrayEquals(new double[] { 0, 0 }, expectedMeasurement.getData());

        Matrix<N2, N1> residual = y.minus(expectedMeasurement);
        assertArrayEquals(new double[] { 1, 0 }, residual.getData());
        System.out.println("residual " + residual);

        Matrix<N2, N1> increment = K.times(residual);
        System.out.println("increment " + increment);

        m_xHat = m_xHat.plus(increment);
        System.out.println("m_xHat " + m_xHat);
        m_P = Matrix.eye(Nat.N2())
                .minus(K.times(C))
                .times(m_P)
                .times(Matrix.eye(Nat.N2()).minus(K.times(C)).transpose())
                .plus(K.times(discR).times(K.transpose()));
        System.out.println("m_P " + m_P);
    }

    /** how does R, Q, and dt affect K? */
    // turn this off because it makes a lot of output.
    // @Test
    public void testKRQDt() {
        System.out.println("KRQDt ======================================");
        System.out.println("     q,      r,     dt, k[0,0]");
        // more dt => more k
        for (double dt = 0.1; dt < 10; dt += 0.1) {
            double r = 1;
            double q = 1;
            Matrix<N2, N2> K = k(q, r, dt);
            System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f\n", q, r, dt, K.get(0, 0));
        }
        for (double q = 0.1; q < 10; q += 0.1) {
            double r = 1;
            double dt = 1;
            Matrix<N2, N2> K = k(q, r, dt);
            System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f\n", q, r, dt, K.get(0, 0));
        }
        for (double r = 0.1; r < 10; r += 0.1) {
            double dt = 1;
            double q = 1;
            Matrix<N2, N2> K = k(q, r, dt);
            System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f\n", q, r, dt, K.get(0, 0));
        }
    }

    private Matrix<N2, N2> k(double q, double r, double dtSeconds) {
        Matrix<N2, N1> stateStdDevs = VecBuilder.fill(q, q);
        Matrix<N2, N2> m_contQ = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stateStdDevs);

        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(r, r);
        Matrix<N2, N2> m_contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), measurementStdDevs);

        Matrix<N2, N1> m_xHat = new Matrix<>(Nat.N2(), Nat.N1());

        Matrix<N2, N2> contA = NumericalJacobian.numericalJacobianX(
                Nat.N2(), Nat.N2(), this::f, m_xHat, new Matrix<>(Nat.N1(), Nat.N1()));

        Matrix<N2, N2> C = NumericalJacobian.numericalJacobianX(
                Nat.N2(), Nat.N2(), this::h, m_xHat, new Matrix<>(Nat.N1(), Nat.N1()));

        Pair<Matrix<N2, N2>, Matrix<N2, N2>> discPair = Discretization.discretizeAQTaylor(contA, m_contQ, dtSeconds);
        Matrix<N2, N2> discA = discPair.getFirst();

        Matrix<N2, N2> discQ = discPair.getSecond();

        Matrix<N2, N2> discR = Discretization.discretizeR(m_contR, dtSeconds);

        Matrix<N2, N2> m_P = Drake.discreteAlgebraicRiccatiEquation(discA.transpose(), C.transpose(), discQ, discR);

        Matrix<N2, N2> S = C.times(m_P).times(C.transpose()).plus(discR);

        Matrix<N2, N2> K = S.transpose().solve(C.times(m_P.transpose())).transpose();

        return K;

    }

}
