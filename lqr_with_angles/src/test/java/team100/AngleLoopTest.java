package team100;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;

/**
 * Demonstrates angle-wrapping with LinearSystemLoop.
 */
public class AngleLoopTest extends KFTestBase {
    /** AngleEKF wraps correctly. */
    final ExtendedKalmanFilter<N2, N1, N1> observer = new AngleEKF(stateStdDevs, measurementStdDevs, kDt);

    /** Works with the loop, using EKF inside. */
    final EKFShim<N2, N1, N1> shim = new EKFShim<>(Nat.N2(), Nat.N1(), plant, stateStdDevs, measurementStdDevs, kDt,
            observer);

    /** AngleLQR wraps correctly. */
    LinearQuadraticRegulator<N2, N1, N1> newController() {
        return new AngleLQR(plant, stateTolerance, controlTolerance, kDt);
    }

    LinearSystemLoop<N2, N1, N1> loop = new LinearSystemLoop<>(plant, controller, shim, 12.0, kDt);

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
        loop.correct(VecBuilder.fill(0));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.002, loop.getXHat(0), kDelta),
                () -> assertEquals(0.229, loop.getXHat(1), kDelta),
                () -> assertEquals(11.465, loop.getU(0), kDelta));

        // update 1: coasting, approx zero output
        loop.correct(VecBuilder.fill(0.002));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.006, loop.getXHat(0), kDelta),
                () -> assertEquals(0.229, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.001, loop.getU(0), kDelta));

        // update 2
        loop.correct(VecBuilder.fill(0.006));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.01, loop.getXHat(0), kDelta),
                () -> assertEquals(0.178, loop.getXHat(1), kDelta),
                () -> assertEquals(-2.559, loop.getU(0), kDelta));

        // update 3
        loop.correct(VecBuilder.fill(0.01));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.014, loop.getXHat(0), kDelta),
                () -> assertEquals(0.126, loop.getXHat(1), kDelta),
                () -> assertEquals(-2.555, loop.getU(0), kDelta));

        // update 4
        loop.correct(VecBuilder.fill(0.014));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.016, loop.getXHat(0), kDelta),
                () -> assertEquals(0.085, loop.getXHat(1), kDelta),
                () -> assertEquals(-2.049, loop.getU(0), kDelta));

        // update 5
        loop.correct(VecBuilder.fill(0.016));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.017, loop.getXHat(0), kDelta),
                () -> assertEquals(0.056, loop.getXHat(1), kDelta),
                () -> assertEquals(-1.453, loop.getU(0), kDelta));

        // update 6
        loop.correct(VecBuilder.fill(0.017));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.018, loop.getXHat(0), kDelta),
                () -> assertEquals(0.037, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.951, loop.getU(0), kDelta));

        // update 7
        loop.correct(VecBuilder.fill(0.018));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.019, loop.getXHat(0), kDelta),
                () -> assertEquals(0.025, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.625, loop.getU(0), kDelta));

        // update 8
        loop.correct(VecBuilder.fill(0.019));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.019, loop.getXHat(0), kDelta),
                () -> assertEquals(0.016, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.439, loop.getU(0), kDelta));

        // update 9
        loop.correct(VecBuilder.fill(0.02));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.02, loop.getXHat(0), kDelta),
                () -> assertEquals(0.009, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.342, loop.getU(0), kDelta));

        // update 10
        loop.correct(VecBuilder.fill(0.02));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(0.02, loop.getXHat(0), kDelta),
                () -> assertEquals(0.005, loop.getXHat(1), kDelta),
                () -> assertEquals(-0.217, loop.getU(0), kDelta));
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
        loop.correct(VecBuilder.fill(-1.0 * Math.PI + 0.01));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(-3.133, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.166, loop.getXHat(1), kDelta),
                () -> assertEquals(-8.324, loop.getU(0), kDelta));

        // update 1: still pushing
        loop.correct(VecBuilder.fill(-3.133));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(-3.137, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.229, loop.getXHat(1), kDelta),
                () -> assertEquals(-3.141, loop.getU(0), kDelta));

        // update 2: slowing down
        loop.correct(VecBuilder.fill(-3.137));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(-3.141, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.191, loop.getXHat(1), kDelta),
                () -> assertEquals(1.895, loop.getU(0), kDelta));

        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        // update 3, note wrapping
        loop.correct(VecBuilder.fill(-3.141));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.138, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.139, loop.getXHat(1), kDelta),
                () -> assertEquals(2.594, loop.getU(0), kDelta));

        // update 4
        loop.correct(VecBuilder.fill(3.138));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.136, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.095, loop.getXHat(1), kDelta),
                () -> assertEquals(2.230, loop.getU(0), kDelta));

        // update 5
        loop.correct(VecBuilder.fill(3.136));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.134, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.063, loop.getXHat(1), kDelta),
                () -> assertEquals(1.606, loop.getU(0), kDelta));

        // update 6
        loop.correct(VecBuilder.fill(3.134));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.133, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.04, loop.getXHat(1), kDelta),
                () -> assertEquals(1.129, loop.getU(0), kDelta));

        // update 7
        loop.correct(VecBuilder.fill(3.133));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.025, loop.getXHat(1), kDelta),
                () -> assertEquals(0.756, loop.getU(0), kDelta));

        // update 8
        loop.correct(VecBuilder.fill(3.132));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.015, loop.getXHat(1), kDelta),
                () -> assertEquals(0.522, loop.getU(0), kDelta));

        // update 9
        loop.correct(VecBuilder.fill(3.132));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.009, loop.getXHat(1), kDelta),
                () -> assertEquals(0.313, loop.getU(0), kDelta));

        // update 10
        loop.correct(VecBuilder.fill(3.132));
        loop.predict(kDt);
        assertAll(
                () -> assertEquals(3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(-0.005, loop.getXHat(1), kDelta),
                () -> assertEquals(0.173, loop.getU(0), kDelta));

    }

}
