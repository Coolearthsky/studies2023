package org.team100.system;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.controller.AngleController;
import org.team100.estimator.ExtendedAngleEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

/**
 * Demonstrates angle-wrapping with LinearSystemLoop.
 */
public class AngleLoopTest {

    static final double kDelta = 0.001;
    static final double kDt = 0.02;

    // MODEL

    // state: x is (angle (rad), angular_velocity (rad/s))
    final Nat<N2> states = Nat.N2();
    // control input: u is (volts)
    final Nat<N1> inputs = Nat.N1();
    // measurement output: y is (angle, velocity)
    final Nat<N2> outputs = Nat.N2();
    // A: angle derivative is velocity
    final Matrix<N2, N2> A = Matrix.mat(states, states)
            .fill(0, 1, //
                    0, 0);
    // B: control input adds velocity
    final Matrix<N2, N1> B = Matrix.mat(states, inputs)
            .fill(0, //
                    1);
    // C: measurement output is angle, velocity
    final Matrix<N2, N2> C = Matrix.mat(outputs, states)
            .fill(1, 0, //
                    0, 1);
    // D: control input does not affect measurement directly
    final Matrix<N2, N1> D = Matrix.mat(outputs, inputs)
            .fill(0, 0);
    final LinearSystem<N2, N1, N2> plant = new LinearSystem<>(A, B, C, D);

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

    final AngleController controller = new AngleController(plant, stateTolerance, controlTolerance, kDt);

    final Vector<N2> angleMeasurementStdDevs = VecBuilder.fill(0.01, 0.1);
    /** AngleEKF wraps correctly. */
    final ExtendedAngleEstimator observer = new ExtendedAngleEstimator(stateStdDevs, angleMeasurementStdDevs, kDt);

    NonlinearSystemLoop loop = new NonlinearSystemLoop(plant, controller, observer, 12.0, kDt);

    @Test
    public void testLoop() {
        // initially, state estimate: at zero, motionless
        loop.reset(VecBuilder.fill(0, 0));
        assertAll(
                () -> assertEquals(0, loop.getXHat(0), kDelta),
                () -> assertEquals(0, loop.getXHat(1), kDelta));

        // try to get to 0.02
        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);
        loop.setNextR(setpoint);

        // initially, push to get started
        loop.correctAngle(0);
        loop.correctVelocity(0);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.002, loop.getXHat(0), kDelta),
                () -> assertEquals(0.229, loop.getXHat(1), kDelta),
                () -> assertEquals(11.465, loop.getU(0), kDelta));

        // update 1: coasting, approx zero output
        loop.correctAngle(0.002);
        loop.correctVelocity(0.229);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.006, loop.getXHat(0), kDelta),
                () -> assertEquals(0.229, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.003, loop.getU(0), kDelta));

        // update 2
        loop.correctAngle(0.006);
        loop.correctVelocity(0.229);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.01, loop.getXHat(0), kDelta),
                () -> assertEquals(0.178, loop.getXHat(1), kDelta),
                () -> assertEquals(-2.566, loop.getU(0), kDelta));

        // update 3
        loop.correctAngle(0.01);
        loop.correctVelocity(0.178);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.014, loop.getXHat(0), kDelta),
                () -> assertEquals(0.126, loop.getXHat(1), kDelta),
                () -> assertEquals(-2.562, loop.getU(0), kDelta));

        // update 4
        loop.correctAngle(0.014);
        loop.correctVelocity(0.126);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.016, loop.getXHat(0), kDelta),
                () -> assertEquals(0.085, loop.getXHat(1), kDelta),
                () -> assertEquals(-2.044, loop.getU(0), kDelta));

        // update 5
        loop.correctAngle(0.016);
        loop.correctVelocity(0.085);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.017, loop.getXHat(0), kDelta),
                () -> assertEquals(0.056, loop.getXHat(1), kDelta),
                () -> assertEquals(-1.448, loop.getU(0), kDelta));

        // update 6
        loop.correctAngle(0.017);
        loop.correctVelocity(0.056);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.018, loop.getXHat(0), kDelta),
                () -> assertEquals(0.037, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.951, loop.getU(0), kDelta));

        // update 7
        loop.correctAngle(0.018);
        loop.correctVelocity(0.037);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.019, loop.getXHat(0), kDelta),
                () -> assertEquals(0.025, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.626, loop.getU(0), kDelta));

        // update 8
        loop.correctAngle(0.019);
        loop.correctVelocity(0.016);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.019, loop.getXHat(0), kDelta),
                () -> assertEquals(0.016, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.417, loop.getU(0), kDelta));

        // update 9
        loop.correctAngle(0.02);
        loop.correctVelocity(0.009);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.02, loop.getXHat(0), kDelta),
                () -> assertEquals(0.009, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.318, loop.getU(0), kDelta));

        // update 10
        loop.correctAngle(0.02);
        loop.correctVelocity(0.005);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.02, loop.getXHat(0), kDelta),
                () -> assertEquals(0.005, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.206, loop.getU(0), kDelta));
    }

    @Test
    public void testWrapping() {
        // start = -pi+0.01
        loop.reset(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertAll(
                () -> assertEquals(-3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(0, loop.getXHat(1), kDelta));

        // goal = pi - 0.01
        Vector<N2> setpoint = VecBuilder.fill(Math.PI - 0.01, 0);
        loop.setNextR(setpoint);

        // initially, push to get started
        loop.correctAngle(-1.0 * Math.PI + 0.01);
        loop.correctVelocity(0);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(-3.133, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.166, loop.getXHat(1), kDelta),
                () -> assertEquals(-8.324, loop.getU(0), kDelta));

        // update 1: still pushing
        loop.correctAngle(-3.133);
        loop.correctVelocity(-0.166);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(-3.137, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.229, loop.getXHat(1), kDelta),
                () -> assertEquals(-3.140, loop.getU(0), kDelta));

        // update 2: slowing down
        loop.correctAngle(-3.137);
        loop.correctVelocity(-0.229);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(-3.141, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.191, loop.getXHat(1), kDelta),
                () -> assertEquals(1.897, loop.getU(0), kDelta));

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3
        loop.correctAngle(-3.141);
        loop.correctVelocity(-0.191);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.138, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.139, loop.getXHat(1), kDelta),
                () -> assertEquals(2.596, loop.getU(0), kDelta));

        // update 4
        loop.correctAngle(3.138);
        loop.correctVelocity(-0.139);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.136, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.095, loop.getXHat(1), kDelta),
                () -> assertEquals(2.224, loop.getU(0), kDelta));

        // update 5
        loop.correctAngle(3.136);
        loop.correctVelocity(-0.095);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.134, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.063, loop.getXHat(1), kDelta),
                () -> assertEquals(1.604, loop.getU(0), kDelta));

        // update 6
        loop.correctAngle(3.134);
        loop.correctVelocity(-0.063);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.133, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.04, loop.getXHat(1), kDelta),
                () -> assertEquals(1.125, loop.getU(0), kDelta));

        // update 7
        loop.correctAngle(3.133);
        loop.correctVelocity(-0.04);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.025, loop.getXHat(1), kDelta),
                () -> assertEquals(0.752, loop.getU(0), kDelta));

        // update 8
        loop.correctAngle(3.132);
        loop.correctVelocity(-0.025);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.015, loop.getXHat(1), kDelta),
                () -> assertEquals(0.516, loop.getU(0), kDelta));

        // update 9
        loop.correctAngle(3.132);
        loop.correctVelocity(-0.015);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.009, loop.getXHat(1), kDelta),
                () -> assertEquals(0.313, loop.getU(0), kDelta));

        // update 10
        loop.correctAngle(3.132);
        loop.correctVelocity(-0.009);
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.005, loop.getXHat(1), kDelta),
                () -> assertEquals(0.176, loop.getU(0), kDelta));

    }

}
