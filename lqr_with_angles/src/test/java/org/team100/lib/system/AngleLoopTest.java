package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.LinearizedLQR;
import org.team100.lib.controller.LinearizedPlantInversionFeedforward;
import org.team100.lib.estimator.NonlinearEstimator;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.NormalDoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Demonstrates angle-wrapping with LinearSystemLoop.
 */
public class AngleLoopTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    final Vector<N2> stateTolerance = VecBuilder.fill(0.01, // angle (rad)
            0.2); // velocity (rad/s)
    final Vector<N1> controlTolerance = VecBuilder.fill(12.0); // output (volts)

    DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
    LinearizedLQR<N2, N1, N2> controller = new LinearizedLQR<>(system, stateTolerance, controlTolerance);
    LinearizedPlantInversionFeedforward<N2, N1, N2> feedforward = new LinearizedPlantInversionFeedforward<>(system);
    NonlinearEstimator<N2, N1, N2> observer = new NonlinearEstimator<>(system, kDt);
    NonlinearSystemLoop<N2, N1, N2> loop = new NonlinearSystemLoop<>(system, controller, feedforward, observer);

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
            loop.correct(VecBuilder.fill(0), system.position());
            loop.correct(VecBuilder.fill(0), system.velocity());
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
            loop.correct(VecBuilder.fill(0.002), system.position());
            loop.correct(VecBuilder.fill(0.229), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.006, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.229, loop.getXHat(1), kDelta),
                    () -> assertEquals(0.006, totalU.get(0, 0), kDelta));
        }
        {
            // update 2
            loop.correct(VecBuilder.fill(0.006), system.position());
            loop.correct(VecBuilder.fill(0.229), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.01, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.178, loop.getXHat(1), kDelta),
                    () -> assertEquals(-2.564, totalU.get(0, 0), kDelta));
        }
        {
            // update 3
            loop.correct(VecBuilder.fill(0.01), system.position());
            loop.correct(VecBuilder.fill(0.178), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.014, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.126, loop.getXHat(1), kDelta),
                    () -> assertEquals(-2.562, totalU.get(0, 0), kDelta));
        }
        {
            // update 4
            loop.correct(VecBuilder.fill(0.014), system.position());
            loop.correct(VecBuilder.fill(0.126), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.016, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.085, loop.getXHat(1), kDelta),
                    () -> assertEquals(-2.044, totalU.get(0, 0), kDelta));
        }
        {
            // update 5
            loop.correct(VecBuilder.fill(0.016), system.position());
            loop.correct(VecBuilder.fill(0.085), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.017, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.056, loop.getXHat(1), kDelta),
                    () -> assertEquals(-1.448, totalU.get(0, 0), kDelta));
        }
        {
            // update 6
            loop.correct(VecBuilder.fill(0.017), system.position());
            loop.correct(VecBuilder.fill(0.056), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.018, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.037, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.951, totalU.get(0, 0), kDelta));
        }
        {
            // update 7
            loop.correct(VecBuilder.fill(0.018), system.position());
            loop.correct(VecBuilder.fill(0.037), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.019, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.025, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.626, totalU.get(0, 0), kDelta));
        }
        {
            // update 8
            loop.correct(VecBuilder.fill(0.019), system.position());
            loop.correct(VecBuilder.fill(0.016), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.019, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.016, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.417, totalU.get(0, 0), kDelta));
        }
        {
            // update 9
            loop.correct(VecBuilder.fill(0.02), system.position());
            loop.correct(VecBuilder.fill(0.009), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(0.02, loop.getXHat(0), kDelta),
                    () -> assertEquals(0.009, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.318, totalU.get(0, 0), kDelta));
        }
        {
            // update 10
            loop.correct(VecBuilder.fill(0.02), system.position());
            loop.correct(VecBuilder.fill(0.005), system.velocity());
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
            loop.correct(VecBuilder.fill(-1.0 * Math.PI + 0.01), system.position());
            loop.correct(VecBuilder.fill(0), system.velocity());
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
            loop.correct(VecBuilder.fill(-3.133), system.position());
            loop.correct(VecBuilder.fill(-0.166), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(-3.138, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.232, loop.getXHat(1), kDelta),
                    () -> assertEquals(-0.194, totalU.get(0, 0), kDelta));
        }
        {
            // update 2: slowing down
            loop.correct(VecBuilder.fill(-3.137), system.position());
            loop.correct(VecBuilder.fill(-0.229), system.velocity());
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
            loop.correct(VecBuilder.fill(-3.141), system.position());
            loop.correct(VecBuilder.fill(-0.191), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(3.138, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.129, loop.getXHat(1), kDelta),
                    () -> assertEquals(2.584, totalU.get(0, 0), kDelta));
        }
        {
            // update 4
            loop.correct(VecBuilder.fill(3.138), system.position());
            loop.correct(VecBuilder.fill(-0.139), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(3.136, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.087, loop.getXHat(1), kDelta),
                    () -> assertEquals(2.089, totalU.get(0, 0), kDelta));
        }
        {
            // update 5
            loop.correct(VecBuilder.fill(3.136), system.position());
            loop.correct(VecBuilder.fill(-0.095), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(3.134, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.058, loop.getXHat(1), kDelta),
                    () -> assertEquals(1.480, totalU.get(0, 0), kDelta));
        }
        {
            // update 6
            loop.correct(VecBuilder.fill(3.134), system.position());
            loop.correct(VecBuilder.fill(-0.063), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(setpoint, rDot, kDt);
            loop.predictState(totalU, kDt);
            assertAll(
                    () -> assertEquals(3.133, loop.getXHat(0), kDelta),
                    () -> assertEquals(-0.037, loop.getXHat(1), kDelta),
                    () -> assertEquals(1.036, totalU.get(0, 0), kDelta));
        }
    }

}
