package org.team100.estimator;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.controller.AngleController;

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

    static final double kDelta = 0.001;
    static final double kDt = 0.02;

    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u
     * 
     * the x jacobian should be constant [0 1 0 0]
     * the u jacobian should be constant [0, 1]
     */
    Matrix<N2, N1> doubleIntegrator(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        // double p = xmat.get(0, 0);
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u;
        return VecBuilder.fill(pdot, vdot);
    }

    // OBSERVER
    // observers are in subclasses; they don't share a superclass. :-(

    // Q: state stdev
    final Vector<N2> stateStdDevs = VecBuilder.fill(0.015, 0.17);
    // R: measurement stdev
    final Vector<N2> measurementStdDevs = VecBuilder.fill(0.01, 0.1);

    // CONTROLLER

    final Vector<N2> stateTolerance = VecBuilder
            .fill(0.01, // angle (rad)
                    0.2); // velocity (rad/s)
    final Vector<N1> controlTolerance = VecBuilder
            .fill(12.0); // output (volts)

    final AngleController controller = new AngleController(this::doubleIntegrator, stateTolerance, controlTolerance,
            kDt);

    /**
     * The derivative of state.
     * 
     * x = (position, velocity)
     * xdot = (velocity, control)
     */
    Matrix<N2, N1> f(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
    }

    /**
     * Both measurements: (position, velocity)
     */
    Matrix<N2, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return x;
    }

    /** AngleEKF wraps correctly. */
    ExtendedAngleEstimator observer = new ExtendedAngleEstimator(this::f, this::h, stateStdDevs, measurementStdDevs,
            kDt);

    @Test
    public void testObserver() {
        // obsP: error covariance
        assertEquals(0.00064, observer.getP(0, 0), 0.0001);
        assertEquals(0.0015, observer.getP(1, 0), 0.0001);
    }

    @Test
    public void testNearZero() {
        // positive setpoint, delta +0.02, push positive

        // controller.reset();
        observer.reset();

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(0, xhat.get(0, 0));
        assertEquals(0, xhat.get(1, 0));

        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);

        // initial: push hard to get started
        Matrix<N1, N1> conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(11.455, conU.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.002);
        xhat = observer.getXhat();
        assertEquals(0.002, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.008, conU.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.006);
        xhat = observer.getXhat();
        assertEquals(0.006, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-2.558, conU.get(0, 0), kDelta);

        // update 3: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.01);
        xhat = observer.getXhat();
        assertEquals(0.010, xhat.get(0, 0), kDelta);
        assertEquals(0.177, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-2.556, conU.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.013);
        xhat = observer.getXhat();
        assertEquals(0.013, xhat.get(0, 0), kDelta);
        assertEquals(0.126, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.971, conU.get(0, 0), kDelta);

        // update 5: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.015);
        xhat = observer.getXhat();
        assertEquals(0.015, xhat.get(0, 0), kDelta);
        assertEquals(0.086, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.38, conU.get(0, 0), kDelta);

        // update 6: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.017);
        xhat = observer.getXhat();
        assertEquals(0.017, xhat.get(0, 0), kDelta);
        assertEquals(0.059, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.976, conU.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.018);
        xhat = observer.getXhat();
        assertEquals(0.018, xhat.get(0, 0), kDelta);
        assertEquals(0.039, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.663, conU.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.019);
        xhat = observer.getXhat();
        assertEquals(0.019, xhat.get(0, 0), kDelta);
        assertEquals(0.026, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.467, conU.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.02);
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.017, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.359, conU.get(0, 0), kDelta);

        // update 10: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.02);
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.01, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.228, conU.get(0, 0), kDelta);

        // update 11: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.02);
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.005, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.131, conU.get(0, 0), kDelta);

        // update 12: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.02);
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.071, conU.get(0, 0), kDelta);

        // update 13: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.02);
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.001, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.037, conU.get(0, 0), kDelta);

        // update 14: pretty much done
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(0.02);
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.019, conU.get(0, 0), kDelta);
    }

    @Test
    public void testNearPiWithoutWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is pi - 0.03
        // so delta is +0.02, should push positive
        // controller.reset();
        observer.reset();
        observer.setXhat(VecBuilder.fill(Math.PI - 0.03, 0));

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(3.112, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        observer.correctAngle(Math.PI - 0.03);
        xhat = observer.getXhat();
        assertEquals(3.112, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);
        Matrix<N1, N1> conU = controller.calculate(xhat, setpoint, kDt);
        // should be same output as above since the motion is the same
        assertEquals(11.455, conU.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.114);
        xhat = observer.getXhat();
        assertEquals(3.114, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.024, conU.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.118);
        xhat = observer.getXhat();
        assertEquals(3.118, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-2.591, conU.get(0, 0), kDelta);

        // update 3: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(3.122);
        xhat = observer.getXhat();
        assertEquals(3.122, xhat.get(0, 0), kDelta);
        assertEquals(0.177, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-2.58, conU.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(3.125);
        xhat = observer.getXhat();
        assertEquals(3.125, xhat.get(0, 0), kDelta);
        assertEquals(0.125, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.987, conU.get(0, 0), kDelta);

        // update 5: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.128);
        xhat = observer.getXhat();
        assertEquals(3.128, xhat.get(0, 0), kDelta);
        assertEquals(0.086, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.465, conU.get(0, 0), kDelta);

        // update 6: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.13);
        xhat = observer.getXhat();
        assertEquals(3.13, xhat.get(0, 0), kDelta);
        assertEquals(0.056, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-1.052, conU.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.131);
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.035, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.719, conU.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.131);
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.021, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.431, conU.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(3.131);
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.012, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.240, conU.get(0, 0), kDelta);

        // update 10: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.131);
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.007, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.126, conU.get(0, 0), kDelta);

        // update 11: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.131);
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.005, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.062, conU.get(0, 0), kDelta);

        // update 12: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.131);
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.028, conU.get(0, 0), kDelta);

        // update 13: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.131);
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.01, conU.get(0, 0), kDelta);

        // update 14: pretty much done
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.131);
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-0.001, conU.get(0, 0), kDelta);
    }

    @Test
    public void testNearPiWithWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary
        // controller.reset();
        observer.reset();
        // starting point is the only difference
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // starting point is the only difference
        observer.correctAngle(-1.0 * Math.PI + 0.01);
        xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);
        Matrix<N1, N1> conU = controller.calculate(xhat, setpoint, kDt);
        // using the EKF this is the correct negative number
        assertEquals(-11.455, conU.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( -3.133);
        xhat = observer.getXhat();
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(-3.133, xhat.get(0, 0), kDelta);
        assertEquals(-0.229, xhat.get(1, 0), kDelta);
        assertEquals(-0.048, conU.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( -3.138);
        xhat = observer.getXhat();
        assertEquals(-3.138, xhat.get(0, 0), kDelta);
        assertEquals(-0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(2.594, conU.get(0, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: still slowing down
        // note boundary crossing here
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.141);
        xhat = observer.getXhat();
        assertEquals(3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.177, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(2.612, conU.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(3.138);
        xhat = observer.getXhat();
        assertEquals(3.138, xhat.get(0, 0), kDelta);
        assertEquals(-0.125, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(2.016, conU.get(0, 0), kDelta);

        // update 5: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle(3.135);
        xhat = observer.getXhat();
        assertEquals(3.135, xhat.get(0, 0), kDelta);
        assertEquals(-0.086, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(1.485, conU.get(0, 0), kDelta);

        // update 6: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.134);
        xhat = observer.getXhat();
        assertEquals(3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.056, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.991, conU.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.133);
        xhat = observer.getXhat();
        assertEquals(3.133, xhat.get(0, 0), kDelta);
        assertEquals(-0.036, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.656, conU.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.132);
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.023, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.460, conU.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.132);
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.014, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.279, conU.get(0, 0), kDelta);

        // update 10: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.132);
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.008, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.155, conU.get(0, 0), kDelta);

        // update 11: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.132);
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.005, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.082, conU.get(0, 0), kDelta);

        // update 12: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.132);
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.040, conU.get(0, 0), kDelta);

        // update 13: almost there
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.132);
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.018, conU.get(0, 0), kDelta);

        // update 14: pretty much done
        observer.predictState(conU.get(0, 0), kDt);
        observer.correctAngle( 3.132);
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint, kDt);
        assertEquals(0.006, conU.get(0, 0), kDelta);
    }

    @Test
    public void testObserverWrappingPredictOnly() {
        // just test the observer prediction across the boundary
        // it just predicts over and over.
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary
        observer.reset();

        // initially, state estimate: at zero, motionless
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // saturate negative-going
        final double conU = -12;

        // update 1
        observer.predictState(conU, kDt);
        xhat = observer.getXhat();
        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.get(1, 0), kDelta);

        // update 2
        observer.predictState(conU, kDt);
        xhat = observer.getXhat();
        assertEquals(-3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: now it wraps around :-)
        // this only works with my wrapping override for predict().
        observer.predictState(conU, kDt);
        xhat = observer.getXhat();
        assertEquals(3.130, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        // update 4:
        observer.predictState(conU, kDt);
        xhat = observer.getXhat();
        assertEquals(3.113, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);
    }

    @Test
    public void testObserverWrappingCorrectVelocityOnly() {
        Vector<N2> stdevs = VecBuilder.fill(0.1, 0.00001);
        observer = new ExtendedAngleEstimator(this::f, this::h, stateStdDevs, stdevs, kDt);
        observer.reset();

        // start in negative territory
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertAll(
                () -> assertEquals(-3.132, observer.getXhat(0), kDelta),
                () -> assertEquals(0, observer.getXhat(1), kDelta));

       //final double conU = -12;

        // note that velocity corrections don't affect position
        // because we're never predicting i.e. time does not pass.
        observer.correctVelocity( -0.240);
        assertAll(
                () -> assertEquals(-3.134, observer.getXhat(0), kDelta),
                () -> assertEquals(-0.239, observer.getXhat(1), kDelta));

        observer.correctVelocity(-0.480);
        assertAll(
                () -> assertEquals(-3.135, observer.getXhat(0), kDelta),
                () -> assertEquals(-0.360, observer.getXhat(1), kDelta));

        observer.correctVelocity(-0.720);
        assertAll(
                () -> assertEquals(-3.136, observer.getXhat(0), kDelta),
                () -> assertEquals(-0.480, observer.getXhat(1), kDelta));
    }

    @Test
    public void testObserverWrappingCorrectPositionOnly() {
        Vector<N2> stdevs = VecBuilder.fill(0.00001, 0.1);
        observer = new ExtendedAngleEstimator(this::f, this::h, stateStdDevs, stdevs, kDt);
        observer.reset();

        // start in negative territory
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertAll(
                () -> assertEquals(-3.132, observer.getXhat(0), kDelta),
                () -> assertEquals(0, observer.getXhat(1), kDelta));

       // final double conU = -12;

        // supply unwrapped corrections
        observer.correctAngle(-3.3);
        // filter wraps it
        assertAll(
                () -> assertEquals(2.983, observer.getXhat(0), kDelta),
                () -> assertEquals(-1.692, observer.getXhat(1), kDelta));

        observer.correctAngle( -3.5);
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
        observer.reset();

        // initially, state estimate: near -pi, motionless
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // saturate negative-going
        final double conU = -12;

        // update 1
        observer.predictState(conU, kDt);
        observer.correctAngle(-3.134);
        xhat = observer.getXhat();
        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.get(1, 0), kDelta);

        // update 2
        observer.predictState(conU, kDt);
        observer.correctAngle(-3.141);
        xhat = observer.getXhat();
        assertEquals(-3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: now it wraps around :-)
        observer.predictState(conU, kDt);
        observer.correctAngle(3.13);
        xhat = observer.getXhat();
        assertEquals(3.130, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        // update 4:
        observer.predictState(conU, kDt);
        observer.correctAngle(3.113);
        xhat = observer.getXhat();
        assertEquals(3.113, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);
    }

}
