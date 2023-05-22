package org.team100.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

/**
 * Demonstrates KalmanFilter in angle-wrapping scenarios, see how the control
 * doesn't work, the kalman filter doesn't work.
 */
public class KFTest {

    static final double kDelta = 0.001;
    static final double kDt = 0.02;

    // MODEL

    // state: x is (angle (rad), angular_velocity (rad/s))
    final Nat<N2> states = Nat.N2();
    // control input: u is (volts)
    final Nat<N1> inputs = Nat.N1();
    // measurement output: y is (angle)
    final Nat<N1> outputs = Nat.N1();
    // A: angle derivative is velocity
    final Matrix<N2, N2> A = Matrix.mat(states, states)
            .fill(0, 1, //
                    0, 0);
    // B: control input adds velocity
    final Matrix<N2, N1> B = Matrix.mat(states, inputs)
            .fill(0, //
                    1);
    // C: measurement output is angle
    final Matrix<N1, N2> C = Matrix.mat(outputs, states)
            .fill(1, 0);
    // D: control input does not affect measurement directly
    final Matrix<N1, N1> D = Matrix.mat(outputs, inputs)
            .fill(0);
    final LinearSystem<N2, N1, N1> plant = new LinearSystem<>(A, B, C, D);

    // OBSERVER
    // observers are in subclasses; they don't share a superclass. :-(

    // Q: state stdev
    final Vector<N2> stateStdDevs = VecBuilder.fill(0.015, 0.17);
    // R: measurement stdev
    final Vector<N1> measurementStdDevs = VecBuilder.fill(0.01);

    // CONTROLLER

    final Vector<N2> stateTolerance = VecBuilder
            .fill(0.01, // angle (rad)
                    0.2); // velocity (rad/s)
    final Vector<N1> controlTolerance = VecBuilder
            .fill(12.0); // output (volts)

    final LinearQuadraticRegulator<N2, N1, N1> controller = new LinearQuadraticRegulator<>(plant, stateTolerance,
            controlTolerance, kDt);

    /** Normal KF does not wrap correctly. */
    final KalmanFilter<N2, N1, N1> observer = new KalmanFilter<>(states, outputs, plant, stateStdDevs,
            measurementStdDevs, kDt);

    @Test
    public void testObserver() {
        // obsK: measurement gain
        Matrix<N2, N1> obsK = observer.getK();
        assertEquals(0.113, obsK.get(0, 0), kDelta);
        assertEquals(0.320, obsK.get(1, 0), kDelta);
    }

    @Test
    public void testNearZero() {
        // example near zero
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
        assertEquals(0.008, conU.get(0, 0), kDelta);

        // update 2: slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.006));
        xhat = observer.getXhat();
        assertEquals(0.006, xhat.get(0, 0), kDelta);
        assertEquals(0.229, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-2.557, conU.get(0, 0), kDelta);

        // update 3: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.01));
        xhat = observer.getXhat();
        assertEquals(0.010, xhat.get(0, 0), kDelta);
        assertEquals(0.177, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-2.554, conU.get(0, 0), kDelta);

        // update 4: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.013));
        xhat = observer.getXhat();
        assertEquals(0.013, xhat.get(0, 0), kDelta);
        assertEquals(0.126, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-1.970, conU.get(0, 0), kDelta);

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
        assertEquals(-0.976, conU.get(0, 0), kDelta);

        // update 7: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.018));
        xhat = observer.getXhat();
        assertEquals(0.018, xhat.get(0, 0), kDelta);
        assertEquals(0.039, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.663, conU.get(0, 0), kDelta);

        // update 8: still slowing down
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.019));
        xhat = observer.getXhat();
        assertEquals(0.019, xhat.get(0, 0), kDelta);
        assertEquals(0.026, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.468, conU.get(0, 0), kDelta);

        // update 9: passing through the setpoint (slowly)
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(0.02));
        xhat = observer.getXhat();
        assertEquals(0.02, xhat.get(0, 0), kDelta);
        assertEquals(0.017, xhat.get(1, 0), kDelta);
        conU = controller.calculate(xhat, setpoint);
        assertEquals(-0.360, conU.get(0, 0), kDelta);

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

        ////////////////////////////////////////////////////////////////////
        //
        // FAIL
        //
        // should be same output as above since the motion is the same,
        // but it's not, it's trying to go all the way around :-(
        assertEquals(3587.385, conU.get(0, 0), kDelta);
    }

    @Test
    public void testObserverWrappingWithoutCorrection() {
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
        final Matrix<N1, N1> conU = VecBuilder.fill(-12);

        // update 1
        observer.predict(conU, kDt);
        xhat = observer.getXhat();
        assertEquals(-3.134, xhat.get(0, 0), kDelta);
        assertEquals(-0.240, xhat.get(1, 0), kDelta);

        // update 2
        observer.predict(conU, kDt);
        xhat = observer.getXhat();
        assertEquals(-3.141, xhat.get(0, 0), kDelta);
        assertEquals(-0.480, xhat.get(1, 0), kDelta);

        ////////////////////////////////////////////////////////////////////
        //
        // FAIL
        //
        // update 3: now it's exceeded -pi
        observer.predict(conU, kDt);
        xhat = observer.getXhat();
        assertEquals(-3.153, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        // update 4: definitely wrong
        observer.predict(conU, kDt);
        xhat = observer.getXhat();
        assertEquals(-3.169, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);
    }

    @Test
    public void testObserverWrappingWithCorrection() {
        // just test the observer across the boundary
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary
        observer.reset();

        // initially, state estimate: at zero, motionless
        observer.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        Matrix<N2, N1> xhat = observer.getXhat();
        assertEquals(-3.132, xhat.get(0, 0), kDelta);
        assertEquals(0, xhat.get(1, 0), kDelta);

        // // starting point is the only difference
        // observer.correct(VecBuilder.fill(0), VecBuilder.fill(-1.0 * Math.PI + 0.01));
        // xhat = observer.getXhat();
        // assertEquals(-3.132, xhat.get(0, 0), kDelta);
        // assertEquals(0, xhat.get(1, 0), kDelta);

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
        // FAIL
        //
        // update 3: now it's exceeded -pi
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(-3.153));
        xhat = observer.getXhat();
        assertEquals(-3.153, xhat.get(0, 0), kDelta);
        assertEquals(-0.720, xhat.get(1, 0), kDelta);

        // update 4: definitely wrong
        observer.predict(conU, kDt);
        observer.correct(conU, VecBuilder.fill(-3.169));
        xhat = observer.getXhat();
        assertEquals(-3.169, xhat.get(0, 0), kDelta);
        assertEquals(-0.960, xhat.get(1, 0), kDelta);

    }

}
