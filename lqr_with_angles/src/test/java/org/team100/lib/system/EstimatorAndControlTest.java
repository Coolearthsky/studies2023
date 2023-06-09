package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.ConstantGainLinearizedLQR;
import org.team100.lib.estimator.NonlinearEstimator;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.NormalDoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Test combination of estimator and controller
 */
public class EstimatorAndControlTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    private RandomVector<N1> y1(double yd) {
        return new RandomVector<>(VecBuilder.fill(yd), VecBuilder.fill(0.1));
    }

    private RandomVector<N2> updateAndCheck(
            NonlinearEstimator<N2, N1, N2> estimator,
            RandomVector<N2> xhat,
            Matrix<N1, N1> u,
            double y,
            Sensor<N2, N1, N1> sensor,
            double x0,
            double x1) {
        xhat = estimator.predictState(xhat, u, kDt);
        xhat = estimator.correct(xhat, y1(y), sensor);
        assertEquals(x0, xhat.x.get(0, 0), kDelta);
        assertEquals(x1, xhat.x.get(1, 0), kDelta);
        return xhat;
    }

    private Matrix<N1, N1> controlAndCheck(
            ConstantGainLinearizedLQR<N2, N1, N2> controller,
            RandomVector<N2> xhat,
            Matrix<N2, N1> setpoint,
            double u0) {
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint);
        assertEquals(u0, u.get(0, 0), kDelta);
        return u;
    }

    @Test
    public void testNearZero() {
        // positive setpoint, delta +0.02, push positive
        final Vector<N2> stateTolerance = VecBuilder
                .fill(0.01, // angle (rad)
                        0.2); // velocity (rad/s)
        final Vector<N1> controlTolerance = VecBuilder
                .fill(12.0); // output (volts)

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
                stateTolerance, controlTolerance, kDt);

        // initially, state estimate: at zero, motionless
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new RandomVector<>(VecBuilder.fill(0, 0), p);
        assertEquals(0, xhat.x.get(0, 0));
        assertEquals(0, xhat.x.get(1, 0));

        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);

        // initial: push hard to get started
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint);
        assertEquals(11.455, u.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        xhat = updateAndCheck(estimator, xhat, u, 0.002, system.position(), 0.002, 0.229);
        u = controlAndCheck(controller, xhat, setpoint, 0.005);

        // update 2: slowing down
        xhat = updateAndCheck(estimator, xhat, u, 0.006, system.position(), 0.006, 0.229);
        u = controlAndCheck(controller, xhat, setpoint, -2.564);

        // update 3: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 0.01, system.position(), 0.01, 0.177);
        u = controlAndCheck(controller, xhat, setpoint, -2.561);

        // update 4: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 0.013, system.position(), 0.013, 0.126);
        u = controlAndCheck(controller, xhat, setpoint, -1.975);

        // update 5: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 0.015, system.position(), 0.015, 0.086);
        u = controlAndCheck(controller, xhat, setpoint, -1.383);

        // update 6: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 0.017, system.position(), 0.017, 0.059);
        u = controlAndCheck(controller, xhat, setpoint, -0.973);

        // update 7: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 0.018, system.position(), 0.018, 0.039);
        u = controlAndCheck(controller, xhat, setpoint, -0.659);

        // update 8: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 0.019, system.position(), 0.019, 0.026);
        u = controlAndCheck(controller, xhat, setpoint, -0.462);

        // update 9: passing through the setpoint (slowly)
        xhat = updateAndCheck(estimator, xhat, u, 0.02, system.position(), 0.02, 0.017);
        u = controlAndCheck(controller, xhat, setpoint, -0.350);

        // update 10: almost there
        xhat = updateAndCheck(estimator, xhat, u, 0.02, system.position(), 0.02, 0.01);
        u = controlAndCheck(controller, xhat, setpoint, -0.223);

        // update 11: almost there
        xhat = updateAndCheck(estimator, xhat, u, 0.02, system.position(), 0.02, 0.005);
        u = controlAndCheck(controller, xhat, setpoint, -0.131);

        // update 12: almost there
        xhat = updateAndCheck(estimator, xhat, u, 0.02, system.position(), 0.02, 0.003);
        u = controlAndCheck(controller, xhat, setpoint, -0.072);

        // update 13: almost there
        xhat = updateAndCheck(estimator, xhat, u, 0.02, system.position(), 0.02, 0.001);
        u = controlAndCheck(controller, xhat, setpoint, -0.039);

        // update 14: pretty much done
        xhat = updateAndCheck(estimator, xhat, u, 0.02, system.position(), 0.02, 0);
        u = controlAndCheck(controller, xhat, setpoint, -0.02);
    }

    @Test
    public void testNearPiWithoutWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is pi - 0.03
        // so delta is +0.02, should push positive

        final Vector<N2> stateTolerance = VecBuilder
                .fill(0.01, // angle (rad)
                        0.2); // velocity (rad/s)
        final Vector<N1> controlTolerance = VecBuilder
                .fill(12.0); // output (volts)

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
                stateTolerance, controlTolerance, kDt);

        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new RandomVector<>(VecBuilder.fill(Math.PI - 0.03, 0), p);

        // initially, state estimate: at zero, motionless
        assertEquals(3.112, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(Math.PI - 0.03), system.position());
        assertEquals(3.112, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint);
        // should be same output as above since the motion is the same
        assertEquals(11.455, u.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        xhat = updateAndCheck(estimator, xhat, u, 3.114, system.position(), 3.114, 0.229);
        u = controlAndCheck(controller, xhat, setpoint, -0.023);

        // update 2: slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.118, system.position(), 3.118, 0.229);
        u = controlAndCheck(controller, xhat, setpoint, -2.591);

        // update 3: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.122, system.position(), 3.122, 0.177);
        u = controlAndCheck(controller, xhat, setpoint, -2.581);

        // update 4: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.125, system.position(), 3.125, 0.125);
        u = controlAndCheck(controller, xhat, setpoint, -1.988);

        // update 5: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.128, system.position(), 3.128, 0.086);
        u = controlAndCheck(controller, xhat, setpoint, -1.462);

        // update 6: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.13, system.position(), 3.13, 0.056);
        u = controlAndCheck(controller, xhat, setpoint, -1.047);

        // update 7: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.131, system.position(), 3.131, 0.035);
        u = controlAndCheck(controller, xhat, setpoint, -0.714);

        // update 8: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.131, system.position(), 3.131, 0.021);
        u = controlAndCheck(controller, xhat, setpoint, -0.431);

        // update 9: passing through the setpoint (slowly)
        xhat = updateAndCheck(estimator, xhat, u, 3.131, system.position(), 3.131, 0.012);
        u = controlAndCheck(controller, xhat, setpoint, -0.242);

        // update 10: almost there
        xhat = updateAndCheck(estimator, xhat, u, 3.131, system.position(), 3.131, 0.007);
        u = controlAndCheck(controller, xhat, setpoint, -0.129);

        // update 11: almost there
        xhat = updateAndCheck(estimator, xhat, u, 3.131, system.position(), 3.131, 0.005);
        u = controlAndCheck(controller, xhat, setpoint, -0.066);

        // update 12: almost there
        xhat = updateAndCheck(estimator, xhat, u, 3.131, system.position(), 3.131, 0.003);
        u = controlAndCheck(controller, xhat, setpoint, -0.031);

        // update 13: almost there
        xhat = updateAndCheck(estimator, xhat, u, 3.131, system.position(), 3.131, 0.003);
        u = controlAndCheck(controller, xhat, setpoint, -0.013);

        // update 14: pretty much done
        xhat = updateAndCheck(estimator, xhat, u, 3.131, system.position(), 3.131, 0.003);
        u = controlAndCheck(controller, xhat, setpoint, -0.004);
    }

    @Test
    public void testNearPiWithWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        final Vector<N2> stateTolerance = VecBuilder
                .fill(0.01, // angle (rad)
                        0.2); // velocity (rad/s)
        final Vector<N1> controlTolerance = VecBuilder
                .fill(12.0); // output (volts)

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
                stateTolerance, controlTolerance, kDt);

        // starting point is the only difference
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new RandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);

        // initially, state estimate: at zero, motionless
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // starting point is the only difference
        xhat = estimator.correct(xhat, y1(-1.0 * Math.PI + 0.01), system.position());

        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);

        Matrix<N1, N1> u = controller.calculate(xhat, setpoint);
        // using the EKF this is the correct negative number
        assertEquals(-11.455, u.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        xhat = updateAndCheck(estimator, xhat, u, -3.133, system.position(), -3.133, -0.229);
        u = controlAndCheck(controller, xhat, setpoint, -0.048);

        // update 2: slowing down
        xhat = updateAndCheck(estimator, xhat, u, -3.138, system.position(), -3.138, -0.229);
        u = controlAndCheck(controller, xhat, setpoint, 2.595);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: still slowing down
        // note boundary crossing here
        xhat = updateAndCheck(estimator, xhat, u, 3.141, system.position(), 3.141, -0.177);
        u = controlAndCheck(controller, xhat, setpoint, 2.612);

        // update 4: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.138, system.position(), 3.138, -0.125);
        u = controlAndCheck(controller, xhat, setpoint, 2.016);

        // update 5: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.135, system.position(), 3.135, -0.086);
        u = controlAndCheck(controller, xhat, setpoint, 1.482);

        // update 6: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.134, system.position(), 3.134, -0.056);
        u = controlAndCheck(controller, xhat, setpoint, 0.989);

        // update 7: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.133, system.position(), 3.133, -0.036);
        u = controlAndCheck(controller, xhat, setpoint, 0.656);

        // update 8: still slowing down
        xhat = updateAndCheck(estimator, xhat, u, 3.132, system.position(), 3.132, -0.023);
        u = controlAndCheck(controller, xhat, setpoint, 0.457);

        // update 9: passing through the setpoint (slowly)
        xhat = updateAndCheck(estimator, xhat, u, 3.132, system.position(), 3.132, -0.014);
        u = controlAndCheck(controller, xhat, setpoint, 0.279);

        // update 10: almost there
        xhat = updateAndCheck(estimator, xhat, u, 3.132, system.position(), 3.132, -0.008);
        u = controlAndCheck(controller, xhat, setpoint, 0.157);

        // update 11: almost there
        xhat = updateAndCheck(estimator, xhat, u, 3.132, system.position(), 3.132, -0.005);
        u = controlAndCheck(controller, xhat, setpoint, 0.084);

        // update 12: almost there
        xhat = updateAndCheck(estimator, xhat, u, 3.132, system.position(), 3.132, -0.003);
        u = controlAndCheck(controller, xhat, setpoint, 0.043);

        // update 13: almost there
        xhat = updateAndCheck(estimator, xhat, u, 3.132, system.position(), 3.132, -0.003);
        u = controlAndCheck(controller, xhat, setpoint, 0.02);

        // update 14: pretty much done
        xhat = updateAndCheck(estimator, xhat, u, 3.132, system.position(), 3.132, -0.003);
        u = controlAndCheck(controller, xhat, setpoint, 0.008);
    }
}
