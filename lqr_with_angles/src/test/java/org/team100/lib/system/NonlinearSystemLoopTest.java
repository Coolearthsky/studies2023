package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.FeedbackControl;
import org.team100.lib.controller.GainCalculator;
import org.team100.lib.controller.LinearizedPlantInversionFeedforward;
import org.team100.lib.estimator.IntegratingPredictor;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Demonstrates angle-wrapping with LinearSystemLoop.
 */
public class NonlinearSystemLoopTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    final Vector<N2> stateTolerance = VecBuilder.fill(0.01, // angle (rad)
            0.2); // velocity (rad/s)
    final Vector<N1> controlTolerance = VecBuilder.fill(12.0); // output (volts)

    DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D();
    
    GainCalculator<N2, N1, N2> gc = new GainCalculator<>(system, stateTolerance, controlTolerance, kDt);
    Matrix<N1, N2> K = gc.getK();
    FeedbackControl<N2, N1, N2> controller = new FeedbackControl<>(system, K);

    IntegratingPredictor<N2, N1,N2> predictor = new IntegratingPredictor<>(system);
    PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);
    LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
    LinearizedPlantInversionFeedforward<N2, N1, N2> feedforward = new LinearizedPlantInversionFeedforward<>(system);
    NonlinearSystemLoop<N2, N1, N2> loop = new NonlinearSystemLoop<>(system, predictor, pointEstimator, pooling, controller, feedforward);

    private RandomVector<N2> yPosition(double yd) {
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, yd); // position
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.1); // TODO: pass variance somehow
        yP.set(1, 1, 1e9); // velocity gets "don't know" variance
        return new AngularRandomVector<>(yx, yP);
    }

    private RandomVector<N2> yVelocity(double yd) {
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(1, 0, yd); // velocity
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 1e9); // position gets "don't know" variance
        yP.set(1, 1, 0.1); // TODO: pass variance somehow
        return new AngularRandomVector<>(yx, yP);
    }

    @Test
    public void testLoop() {
        // initially, state estimate: at zero, motionless
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(0, 0), p);
        assertArrayEquals(new double[] { 0, 0 }, xhat.x.getData(), kDelta);

        // try to get to 0.02
        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);
        // for this example we're doing a step function
        // in reality we would use a constrained trajectory
        Vector<N2> rDot = VecBuilder.fill(1, 0);

        {
            // initially, push to get started
            xhat = loop.correct(xhat, yPosition(0));
            xhat = loop.correct(xhat, yVelocity(0));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(11.455, totalU.get(0, 0), kDelta);

            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.002, 0.229 }, xhat.x.getData(), kDelta);
        }
        // no more change in setpoint after this.
        rDot = VecBuilder.fill(0, 0);

        {
            xhat = loop.correct(xhat, yPosition(0.002));
            xhat = loop.correct(xhat, yVelocity(0.229));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(0.1, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.006, 0.231 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(0.006));
            xhat = loop.correct(xhat, yVelocity(0.229));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-2.266, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.01, 0.184 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(0.010));
            xhat = loop.correct(xhat, yVelocity(0.178));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-2.281, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.014, 0.134 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(0.014));
            xhat = loop.correct(xhat, yVelocity(0.126));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-2.125, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.016, 0.086 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(0.016));
            xhat = loop.correct(xhat, yVelocity(0.085));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-1.474, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.017, 0.056 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(0.017));
            xhat = loop.correct(xhat, yVelocity(0.056));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.817, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.018, 0.039 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(0.018));
            xhat = loop.correct(xhat, yVelocity(0.037));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.531, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.019, 0.027 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(0.019));
            xhat = loop.correct(xhat, yVelocity(0.016));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.210, totalU.get(0, 0), kDelta); // ??
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.019, 0.015 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(0.020));
            xhat = loop.correct(xhat, yVelocity(0.009));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.353, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.02, 0.003 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(0.020));
            xhat = loop.correct(xhat, yVelocity(0.005));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.196, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.02, 0.001 }, xhat.x.getData(), kDelta);
        }
    }

    @Test
    public void testWrapping() {
        // start = -pi+0.01
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);

        // goal = pi - 0.01
        Vector<N2> setpoint = VecBuilder.fill(Math.PI - 0.01, 0);
        // for this example we're doing a step function
        // in reality we would use a constrained trajectory
        Vector<N2> rDot = VecBuilder.fill((Math.PI - 0.01) / 0.02, 0);

        {
            xhat = loop.correct(xhat, yPosition(-1.0 * Math.PI + 0.01));
            xhat = loop.correct(xhat, yVelocity(0));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-11.455, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { -3.133, -0.229 }, xhat.x.getData(), kDelta);
        }
        rDot = VecBuilder.fill(0, 0);
        {
            xhat = loop.correct(xhat, yPosition(-3.133));
            xhat = loop.correct(xhat, yVelocity(-0.166));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-2.358, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { -3.138, -0.230 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(-3.137));
            xhat = loop.correct(xhat, yVelocity(-0.229));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(1.877, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { -3.141, -0.191 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(-3.141));
            xhat = loop.correct(xhat, yVelocity(-0.191));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(2.458, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 3.138, -0.142 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(3.138));
            xhat = loop.correct(xhat, yVelocity(-0.139));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(2.416, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 3.136, -0.091 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(3.136));
            xhat = loop.correct(xhat, yVelocity(-0.095));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(1.665, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 3.134, -0.061 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, yPosition(3.134));
            xhat = loop.correct(xhat, yVelocity(-0.063));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(1.330, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 3.133, -0.036 }, xhat.x.getData(), kDelta);
        }
    }
}
