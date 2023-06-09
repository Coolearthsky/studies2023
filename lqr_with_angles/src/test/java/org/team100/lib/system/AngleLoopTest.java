package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.ConstantGainLinearizedLQR;
import org.team100.lib.controller.LinearizedPlantInversionFeedforward;
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
 * Demonstrates angle-wrapping with LinearSystemLoop.
 */
public class AngleLoopTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    final Vector<N2> stateTolerance = VecBuilder.fill(0.01, // angle (rad)
            0.2); // velocity (rad/s)
    final Vector<N1> controlTolerance = VecBuilder.fill(12.0); // output (volts)

    DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
    ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
            stateTolerance, controlTolerance, kDt);
    LinearizedPlantInversionFeedforward<N2, N1, N2> feedforward = new LinearizedPlantInversionFeedforward<>(system);
    NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(system, kDt);
    NonlinearSystemLoop<N2, N1, N2> loop = new NonlinearSystemLoop<>(system, controller, feedforward, estimator);

    private RandomVector<N1> y1(double yd) {
        return new RandomVector<>(VecBuilder.fill(yd), VecBuilder.fill(0.1));
    }

    @Test
    public void testLoop() {
        // initially, state estimate: at zero, motionless
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new RandomVector<>(VecBuilder.fill(0, 0), p);
        assertEquals(0, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // try to get to 0.02
        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);
        // for this example we're doing a step function
        // in reality we would use a constrained trajectory
        Vector<N2> rDot = VecBuilder.fill(1, 0);

        {
            // initially, push to get started
            xhat = loop.correct(xhat, y1(0), system.position());
            xhat = loop.correct(xhat, y1(0), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.002, xhat.x.get(0, 0), kDelta);
            assertEquals(0.229, xhat.x.get(1, 0), kDelta);
            assertEquals(11.455, totalU.get(0, 0), kDelta);
        }
        // no more change in setpoint after this.
        rDot = VecBuilder.fill(0, 0);

        {
            // update 1: coasting, approx zero output
            xhat = loop.correct(xhat, y1(0.002), system.position());
            assertArrayEquals(new double[] { 0.002, 0.229 }, xhat.x.getData(), kDelta);
            assertArrayEquals(new double[] { 0.133, 0, 0, 0.266 }, xhat.P.getData(), kDelta);
            xhat = loop.correct(xhat, y1(0.229), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.006, xhat.x.get(0, 0), kDelta);
            assertEquals(0.229, xhat.x.get(1, 0), kDelta);
            assertEquals(0.006, totalU.get(0, 0), kDelta);
        }
        {
            // update 2
            xhat = loop.correct(xhat, y1(0.006), system.position());
            xhat = loop.correct(xhat, y1(0.229), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.01, xhat.x.get(0, 0), kDelta);
            assertEquals(0.178, xhat.x.get(1, 0), kDelta);
            assertEquals(-2.564, totalU.get(0, 0), kDelta);
        }
        {
            // update 3
            xhat = loop.correct(xhat, y1(0.01), system.position());
            xhat = loop.correct(xhat, y1(0.178), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.014, xhat.x.get(0, 0), kDelta);
            assertEquals(0.126, xhat.x.get(1, 0), kDelta);
            assertEquals(-2.562, totalU.get(0, 0), kDelta);
        }
        {
            // update 4
            xhat = loop.correct(xhat, y1(0.014), system.position());
            xhat = loop.correct(xhat, y1(0.126), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.016, xhat.x.get(0, 0), kDelta);
            assertEquals(0.085, xhat.x.get(1, 0), kDelta);
            assertEquals(-2.044, totalU.get(0, 0), kDelta);
        }
        {
            // update 5
            xhat = loop.correct(xhat, y1(0.016), system.position());
            xhat = loop.correct(xhat, y1(0.085), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.017, xhat.x.get(0, 0), kDelta);
            assertEquals(0.056, xhat.x.get(1, 0), kDelta);
            assertEquals(-1.448, totalU.get(0, 0), kDelta);
        }
        {
            // update 6
            xhat = loop.correct(xhat, y1(0.017), system.position());
            xhat = loop.correct(xhat, y1(0.056), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.018, xhat.x.get(0, 0), kDelta);
            assertEquals(0.037, xhat.x.get(1, 0), kDelta);
            assertEquals(-0.951, totalU.get(0, 0), kDelta);
        }
        {
            // update 7
            xhat = loop.correct(xhat, y1(0.018), system.position());
            xhat = loop.correct(xhat, y1(0.037), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.019, xhat.x.get(0, 0), kDelta);
            assertEquals(0.025, xhat.x.get(1, 0), kDelta);
            assertEquals(-0.626, totalU.get(0, 0), kDelta);
        }
        {
            // update 8
            xhat = loop.correct(xhat, y1(0.019), system.position());
            xhat = loop.correct(xhat, y1(0.016), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.019, xhat.x.get(0, 0), kDelta);
            assertEquals(0.016, xhat.x.get(1, 0), kDelta);
            assertEquals(-0.415, totalU.get(0, 0), kDelta);
        }
        {
            // update 9
            xhat = loop.correct(xhat, y1(0.02), system.position());
            xhat = loop.correct(xhat, y1(0.009), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.02, xhat.x.get(0, 0), kDelta);
            assertEquals(0.01, xhat.x.get(1, 0), kDelta);
            assertEquals(-0.316, totalU.get(0, 0), kDelta);
        }
        {
            // update 10
            xhat = loop.correct(xhat, y1(0.02), system.position());
            xhat = loop.correct(xhat, y1(0.005), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(0.02, xhat.x.get(0, 0), kDelta);
            assertEquals(0.005, xhat.x.get(1, 0), kDelta);
            assertEquals(-0.206, totalU.get(0, 0), kDelta);
        }
    }

    @Test
    public void testWrapping() {
        // start = -pi+0.01
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new RandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // goal = pi - 0.01
        Vector<N2> setpoint = VecBuilder.fill(Math.PI - 0.01, 0);
        // for this example we're doing a step function
        // in reality we would use a constrained trajectory
        Vector<N2> rDot = VecBuilder.fill((Math.PI - 0.01) / 0.02, 0);

        {
            // initially, push to get started
            xhat = loop.correct(xhat, y1(-1.0 * Math.PI + 0.01), system.position());
            xhat = loop.correct(xhat, y1(0), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(-3.133, xhat.x.get(0, 0), kDelta);
            assertEquals(-0.229, xhat.x.get(1, 0), kDelta);
            assertEquals(-11.455, totalU.get(0, 0), kDelta);
        }
        rDot = VecBuilder.fill(0, 0);
        {
            // update 1: still pushing
            xhat = loop.correct(xhat, y1(-3.133), system.position());
            xhat = loop.correct(xhat, y1(-0.166), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(-3.138, xhat.x.get(0, 0), kDelta);
            assertEquals(-0.232, xhat.x.get(1, 0), kDelta);
            assertEquals(-0.204, totalU.get(0, 0), kDelta);
        }
        {
            // update 2: slowing down
            xhat = loop.correct(xhat, y1(-3.137), system.position());
            xhat = loop.correct(xhat, y1(-0.229), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(3.141, xhat.x.get(0, 0), kDelta);
            assertEquals(-0.181, xhat.x.get(1, 0), kDelta);
            assertEquals(2.518, totalU.get(0, 0), kDelta);
        }
        ////////////////////////////////////////////////////////////////////
        //
        // SUCCESS
        //
        {
            // update 3
            xhat = loop.correct(xhat, y1(-3.141), system.position());
            xhat = loop.correct(xhat, y1(-0.191), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(3.138, xhat.x.get(0, 0), kDelta);
            assertEquals(-0.129, xhat.x.get(1, 0), kDelta);
            assertEquals(2.59, totalU.get(0, 0), kDelta);
        }
        {
            // update 4
            xhat = loop.correct(xhat, y1(3.138), system.position());
            xhat = loop.correct(xhat, y1(-0.139), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(3.136, xhat.x.get(0, 0), kDelta);
            assertEquals(-0.087, xhat.x.get(1, 0), kDelta);
            assertEquals(2.092, totalU.get(0, 0), kDelta);
        }
        {
            // update 5
            xhat = loop.correct(xhat, y1(3.136), system.position());
            xhat = loop.correct(xhat, y1(-0.095), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(3.134, xhat.x.get(0, 0), kDelta);
            assertEquals(-0.058, xhat.x.get(1, 0), kDelta);
            assertEquals(1.482, totalU.get(0, 0), kDelta);
        }
        {
            // update 6
            xhat = loop.correct(xhat, y1(3.134), system.position());
            xhat = loop.correct(xhat, y1(-0.063), system.velocity());
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertEquals(3.133, xhat.x.get(0, 0), kDelta);
            assertEquals(-0.037, xhat.x.get(1, 0), kDelta);
            assertEquals(1.036, totalU.get(0, 0), kDelta);
        }
    }

}
