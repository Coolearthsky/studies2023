package team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Demonstrates ExtendedKalmanFilter in angle-wrapping scenarios, and also a
 * custom LQR that implements wrapping as well.
 * 
 * The model for this test is a 1-DOF arm without gravity.
 * 
 * state: (angle, angular velocity)
 * measurement: angle
 * output: torque, i guess?
 */
public class EKFTest extends KFTestBase {

    /** EKF allows wrapping using the AngleStatistics functions below. */
    final ExtendedKalmanFilter<N2, N1, N1> observer = new ExtendedKalmanFilter<>(
            states,
            inputs,
            outputs,
            EKFTest::f,
            EKFTest::h,
            Q, R,
            AngleStatistics.angleResidual(0),
            AngleStatistics.angleAdd(0), // zero'th row is the angle
            kDt);

    /** AngleLQR implements wrapping */
    LinearQuadraticRegulator<N2, N1, N1> newController() {
        return new AngleLQR<>(
                plant,
                stateTolerance,
                controlTolerance,
                kDt);
    }

    /**
     * f is the derivative of state, like A and B
     */
    private static Matrix<N2, N1> f(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        // position derivative is velocity
        // velocity derivative is control
        return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
    }

    /**
     * h is the measurement, like C and D
     */
    private static Matrix<N1, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        // measurement is position (angle)
        return VecBuilder.fill(x.get(0, 0));
    }

    @Test
    public void testObserver() {
        // obsP: error covariance
        Matrix<N2, N2> obsP = observer.getP();
        assertEquals(0.00064, obsP.get(0, 0), 0.0001);
        assertEquals(0.0018, obsP.get(1, 0), 0.0001);
    }

    @Test
    public void testNearZero() {
        // positive setpoint, delta +0.02, push positive

        controller.reset();
        observer.reset();

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(0, xhat.get(0, 0));
        assertEquals(0, xhat.get(1, 0));

        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);

        // initial: push hard to get started
        Matrix<N1, N1> conU = controller.calculate(xhat, setpoint);
        assertEquals(11.455, conU.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.002));
        xhat = observer.getXhat();
        assertEquals(0.002, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.011, conU.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.006));
        xhat = observer.getXhat();
        assertEquals(0.006, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-2.551, conU.get(0, 0), kDelta);

        // update 3: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.01));
        xhat = observer.getXhat();
        assertEquals(0.010, xhat.get(0, 0), kDelta);
        assertEquals(0.177, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-2.551, conU.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.013));
        xhat = observer.getXhat();
        assertEquals(0.013, xhat.get(0, 0), kDelta);
        assertEquals(0.126, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-1.969, conU.get(0, 0), kDelta);

        // update 5: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.015));
        xhat = observer.getXhat();
        assertEquals(0.015, xhat.get(0, 0), kDelta);
        assertEquals(0.086, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-1.379, conU.get(0, 0), kDelta);

        // update 6: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.017));
        xhat = observer.getXhat();
        assertEquals(0.017, xhat.get(0, 0), kDelta);
        assertEquals(0.059, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.979, conU.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.018));
        xhat = observer.getXhat();
        assertEquals(0.018, xhat.get(0, 0), kDelta);
        assertEquals(0.039, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.666, conU.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.019));
        xhat = observer.getXhat();
        assertEquals(0.019, xhat.get(0, 0), kDelta);
        assertEquals(0.026, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.471, conU.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.02));
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.017, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.363, conU.get(0, 0), kDelta);

        // update 10: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.02));
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.01, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.228, conU.get(0, 0), kDelta);

        // update 11: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.02));
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.005, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.131, conU.get(0, 0), kDelta);

        // update 12: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.02));
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.071, conU.get(0, 0), kDelta);

        // update 13: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.02));
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.001, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.037, conU.get(0, 0), kDelta);

        // update 14: pretty much done
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.02));
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.019, conU.get(0, 0), kDelta);
    }

    @Test
    public void testNearPiWithoutWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is pi - 0.03
        // so delta is +0.02, should push positive
        controller.reset();
        observer.reset();
        observer.setXhat(VecBuilder.fill(Math.PI - 0.03, 0));

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(3.112, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        observer.correct(VecBuilder.fill(0), VecBuilder.fill(Math.PI - 0.03));
        xhat = observer.getXhat();
        assertEquals(3.112, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(states, Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);
        Matrix<N1, N1> conU = controller.calculate(xhat, setpoint);
        // should be same output as above since the motion is the same
        assertEquals(11.455, conU.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.114));
        xhat = observer.getXhat();
        assertEquals(3.114, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.024, conU.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.118));
        xhat = observer.getXhat();
        assertEquals(3.118, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-2.587, conU.get(0, 0), kDelta);

        // update 3: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.122));
        xhat = observer.getXhat();
        assertEquals(3.122, xhat.get(0, 0), kDelta);
        assertEquals(0.177, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-2.577, conU.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.125));
        xhat = observer.getXhat();
        assertEquals(3.125, xhat.get(0, 0), kDelta);
        assertEquals(0.125, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-1.985, conU.get(0, 0), kDelta);

        // update 5: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.128));
        xhat = observer.getXhat();
        assertEquals(3.128, xhat.get(0, 0), kDelta);
        assertEquals(0.086, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-1.469, conU.get(0, 0), kDelta);

        // update 6: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.13));
        xhat = observer.getXhat();
        assertEquals(3.13, xhat.get(0, 0), kDelta);
        assertEquals(0.056, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-1.059, conU.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.131));
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.035, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.724, conU.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.131));
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.021, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.431, conU.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.131));
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.012, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.238, conU.get(0, 0), kDelta);

        // update 10: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.131));
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.007, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.123, conU.get(0, 0), kDelta);

        // update 11: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.131));
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.005, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.06, conU.get(0, 0), kDelta);

        // update 12: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.131));
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.027, conU.get(0, 0), kDelta);

        // update 13: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.131));
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.01, conU.get(0, 0), kDelta);

        // update 14: pretty much done
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.131));
        xhat = observer.getXhat();
        assertEquals(3.131, xhat.get(0, 0), kDelta);
        assertEquals(0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.001, conU.get(0, 0), kDelta);
    }

    @Test
    public void testNearPiWithWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary
        controller.reset();
        observer.reset();
        // starting point is the only difference
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // starting point is the only difference
        observer.correct(VecBuilder.fill(0), VecBuilder.fill(-1.0 * Math.PI + 0.01));
        xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(states, Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);
        Matrix<N1, N1> conU = controller.calculate(xhat, setpoint);
        // using the EKF this is the correct negative number
        assertEquals(-11.455, conU.get(0, 0), kDelta);

        // update 1: coasting, approx zero output
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(-3.133));
        xhat = observer.getXhat();
        assertEquals(-3.133, xhat.get(0, 0), kDelta);
        assertEquals(-0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.055, conU.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(-3.138));
        xhat = observer.getXhat();
        assertEquals(-3.138, xhat.get(0, 0), kDelta);
        assertEquals(-0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(2.592, conU.get(0, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: still slowing down
        // note boundary crossing here
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.141));
        xhat = observer.getXhat();
        assertEquals(3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.177, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(2.612, conU.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.138));
        xhat = observer.getXhat();
        assertEquals(3.138, xhat.get(0, 0), kDelta);
        assertEquals(-0.125, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(2.016, conU.get(0, 0), kDelta);

        // update 5: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.135));
        xhat = observer.getXhat();
        assertEquals(3.135, xhat.get(0, 0), kDelta);
        assertEquals(-0.086, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(1.490, conU.get(0, 0), kDelta);

        // update 6: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.134));
        xhat = observer.getXhat();
        assertEquals(3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.056, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.993, conU.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.133));
        xhat = observer.getXhat();
        assertEquals(3.133, xhat.get(0, 0), kDelta);
        assertEquals(-0.036, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.658, conU.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.132));
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.023, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.462, conU.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.132));
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.014, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.279, conU.get(0, 0), kDelta);

        // update 10: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.132));
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.008, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.155, conU.get(0, 0), kDelta);

        // update 11: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.132));
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.005, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.08, conU.get(0, 0), kDelta);

        // update 12: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.132));
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.039, conU.get(0, 0), kDelta);

        // update 13: almost there
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.132));
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.017, conU.get(0, 0), kDelta);

        // update 14: pretty much done
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(3.132));
        xhat = observer.getXhat();
        assertEquals(3.132, xhat.get(0, 0), kDelta);
        assertEquals(-0.003, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(0.006, conU.get(0, 0), kDelta);
    }

    @Test
    public void testObserverWrapping() {
        // just test the observer across the boundary
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary
        observer.reset();
        // starting point is the only difference
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));

        // initially, state estimate: at zero, motionless
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // starting point is the only difference
        observer.correct(VecBuilder.fill(0), VecBuilder.fill(-1.0 * Math.PI + 0.01));
        xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // saturate negative-going
        final Matrix<N1, N1> conU = VecBuilder.fill(-12);

        // update 1
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(-3.134));
        xhat = observer.getXhat();
        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.get(1, 0), kDelta);

        // update 2
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(-3.141));
        xhat = observer.getXhat();
        assertEquals(-3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3: now it wraps around :-)
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(-3.153));
        xhat = observer.getXhat();
        assertEquals(3.130, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        // update 4: definitely wrong
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(-3.169));
        xhat = observer.getXhat();
        assertEquals(3.113, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);
    }

}
