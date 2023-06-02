package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.AngleController;
import org.team100.lib.controller.ImmutableControlAffinePlantInversionFeedforward;
import org.team100.lib.estimator.ExtendedAngleEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Demonstrates angle-wrapping with LinearSystemLoop.
 */
public class AngleLoopTest {
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
    final Vector<N1> measurementStdDevs = VecBuilder.fill(0.01);

    // CONTROLLER

    final Vector<N2> stateTolerance = VecBuilder
            .fill(0.01, // angle (rad)
                    0.2); // velocity (rad/s)
    final Vector<N1> controlTolerance = VecBuilder
            .fill(12.0); // output (volts)

    final AngleController controller = new AngleController(this::doubleIntegrator, stateTolerance, controlTolerance,
            kDt);

    final Vector<N2> angleMeasurementStdDevs = VecBuilder.fill(0.01, 0.1);

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

    Matrix<N1, N1> desaturate(Matrix<N1, N1> u) {
        return StateSpaceUtil.desaturateInputVector(u, 12.0);
    }

    /** AngleEKF wraps correctly. */
    final ExtendedAngleEstimator observer = new ExtendedAngleEstimator(
            this::f,
            this::h,
            stateStdDevs,
            angleMeasurementStdDevs,
            kDt);

    ImmutableControlAffinePlantInversionFeedforward<N2, N1> feedforward = new ImmutableControlAffinePlantInversionFeedforward<>(
            Nat.N2(),
            Nat.N1(),
            this::f);
    NonlinearSystemLoop loop = new NonlinearSystemLoop(controller, feedforward, observer, this::desaturate);

    @Test
    public void testLoop() {
        // initially, state estimate: at zero, motionless
        loop.setXhat(VecBuilder.fill(0, 0));
        assertAll(
                () -> assertEquals(0, loop.getXHat(0), kDelta),
                () -> assertEquals(0, loop.getXHat(1), kDelta));

        // try to get to 0.02
        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);
        // for this example we're doing a step function
        // in reality we would use a constrained trajectory
        Vector<N2> rDot = VecBuilder.fill(1, 0);

        {
            // initially, push to get started
            loop.correctAngle(0);
            loop.correctVelocity(0);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.002, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.229, loop.getXHat(1), kDelta),
                    () -> assertEquals(11.455, totalU.get(0, 0), kDelta));
        }
        // no more change in setpoint after this.
        rDot = VecBuilder.fill(0, 0);

        {
            // update 1: coasting, approx zero output
            loop.correctAngle(0.002);
            loop.correctVelocity(0.229);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.006, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.229, loop.getXHat(1), kDelta),
                    () -> assertEquals(0.006, totalU.get(0, 0), kDelta));
        }
        {
            // update 2
            loop.correctAngle(0.006);
            loop.correctVelocity(0.229);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.01, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.178, loop.getXHat(1), kDelta),
                    () -> assertEquals(-2.564, totalU.get(0, 0), kDelta));
        }
        {
            // update 3
            loop.correctAngle(0.01);
            loop.correctVelocity(0.178);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.014, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.126, loop.getXHat(1), kDelta),
                    () -> assertEquals(-2.562, totalU.get(0, 0), kDelta));
        }
        {
            // update 4
            loop.correctAngle(0.014);
            loop.correctVelocity(0.126);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.016, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.085, loop.getXHat(1), kDelta),
                    () -> assertEquals(-2.044, totalU.get(0, 0), kDelta));
        }
        {
            // update 5
            loop.correctAngle(0.016);
            loop.correctVelocity(0.085);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.017, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.056, loop.getXHat(1), kDelta),
                    () -> assertEquals(-1.448, totalU.get(0, 0), kDelta));
        }
        {
            // update 6
            loop.correctAngle(0.017);
            loop.correctVelocity(0.056);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.018, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.037, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.951, totalU.get(0, 0), kDelta));
        }
        {
            // update 7
            loop.correctAngle(0.018);
            loop.correctVelocity(0.037);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.019, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.025, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.626, totalU.get(0, 0), kDelta));
        }
        {
            // update 8
            loop.correctAngle(0.019);
            loop.correctVelocity(0.016);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.019, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.016, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.417, totalU.get(0, 0), kDelta));
        }
        {
            // update 9
            loop.correctAngle(0.02);
            loop.correctVelocity(0.009);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.02, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.009, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.318, totalU.get(0, 0), kDelta));
        }
        {
            // update 10
            loop.correctAngle(0.02);
            loop.correctVelocity(0.005);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.02, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.005, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.206, totalU.get(0, 0), kDelta));
        }
    }

    @Test
    public void testWrapping() {
        // start = -pi+0.01
        loop.setXhat(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0));
        assertAll(
                () -> assertEquals(-3.132, loop.getXHat(0), kDelta),
                () -> assertEquals(0, loop.getXHat(1), kDelta));

        // goal = pi - 0.01
        Vector<N2> setpoint = VecBuilder.fill(Math.PI - 0.01, 0);
        // for this example we're doing a step function
        // in reality we would use a constrained trajectory
        Vector<N2> rDot = VecBuilder.fill((Math.PI - 0.01) / 0.02, 0);

        {
            // initially, push to get started
            loop.correctAngle(-1.0 * Math.PI + 0.01);
            loop.correctVelocity(0);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(-3.133, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.229, loop.getXHat(1), kDelta),
                    () -> assertEquals(-11.455, totalU.get(0, 0), kDelta));
        }
        rDot = VecBuilder.fill(0, 0);
        {
            // update 1: still pushing
            loop.correctAngle(-3.133);
            loop.correctVelocity(-0.166);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(-3.138, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.232, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.194, totalU.get(0, 0), kDelta));
        }
        {
            // update 2: slowing down
            loop.correctAngle(-3.137);
            loop.correctVelocity(-0.229);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(3.141, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.181, loop.getXHat(1), kDelta),
                    () -> assertEquals(2.518, totalU.get(0, 0), kDelta));
        }
        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        {
            // update 3
            loop.correctAngle(-3.141);
            loop.correctVelocity(-0.191);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(3.138, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.129, loop.getXHat(1), kDelta),
                    () -> assertEquals(2.584, totalU.get(0, 0), kDelta));
        }
        {
            // update 4
            loop.correctAngle(3.138);
            loop.correctVelocity(-0.139);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(3.136, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.087, loop.getXHat(1), kDelta),
                    () -> assertEquals(2.089, totalU.get(0, 0), kDelta));
        }
        {
            // update 5
            loop.correctAngle(3.136);
            loop.correctVelocity(-0.095);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(3.134, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.058, loop.getXHat(1), kDelta),
                    () -> assertEquals(1.480, totalU.get(0, 0), kDelta));
        }
        {
            // update 6
            loop.correctAngle(3.134);
            loop.correctVelocity(-0.063);
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(3.133, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.037, loop.getXHat(1), kDelta),
                    () -> assertEquals(1.036, totalU.get(0, 0), kDelta));
        }
    }

}
