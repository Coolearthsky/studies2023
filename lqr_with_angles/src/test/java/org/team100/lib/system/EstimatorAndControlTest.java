package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.FeedbackControl;
import org.team100.lib.controller.GainCalculator;
import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

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

    // for this test, zero noise
    WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0, 0);
    MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
    final DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w,v);
    final ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(system);
    final PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);
    final LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
    // angle (rad), velocity (rad/s)
    final Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
    // output (volts)
    final Vector<N1> controlTolerance = VecBuilder.fill(12.0);
    GainCalculator<N2, N1, N2> gc = new GainCalculator<>(system, stateTolerance, controlTolerance, kDt);
    Matrix<N1, N2> K = gc.getK();
    final FeedbackControl<N2, N1, N2> controller = new FeedbackControl<>(system, K);

    private RandomVector<N2> updateAndCheck(
            RandomVector<N2> xhat,
            Matrix<N1, N1> u,
            double y,
            double x0,
            double x1) {
        xhat = predictor.predictWithNoise(xhat, u, kDt);

        RandomVector<N2> x = pointEstimator.stateForMeasurementWithZeroU(system.position(y));
        xhat = pooling.fuse(x, xhat);
        assertEquals(x0, xhat.x.get(0, 0), kDelta);
        assertEquals(x1, xhat.x.get(1, 0), kDelta);
        return xhat;
    }

    private Matrix<N1, N1> controlAndCheck(
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

        // initially, state estimate: at zero, motionless
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(0, 0), p);
        assertEquals(0, xhat.x.get(0, 0));
        assertEquals(0, xhat.x.get(1, 0));

        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);

        Matrix<N1, N1> u = controlAndCheck(xhat, setpoint, 11.455);

        xhat = updateAndCheck(xhat, u, 0.002, 0.002, 0.229);
        u = controlAndCheck(xhat, setpoint, 0.137);

        xhat = updateAndCheck(xhat, u, 0.006, 0.006, 0.232);
        u = controlAndCheck(xhat, setpoint, -2.390);

        xhat = updateAndCheck(xhat, u, 0.01, 0.01, 0.184);
        u = controlAndCheck(xhat, setpoint, -2.529);

        xhat = updateAndCheck(xhat, u, 0.013, 0.013, 0.133);
        u = controlAndCheck(xhat, setpoint, -2.001);

        xhat = updateAndCheck(xhat, u, 0.015, 0.015, 0.093);
        u = controlAndCheck(xhat, setpoint, -1.400);

        xhat = updateAndCheck(xhat, u, 0.017, 0.017, 0.065);
        u = controlAndCheck(xhat, setpoint, -1.127);

        xhat = updateAndCheck(xhat, u, 0.018, 0.018, 0.043);
        u = controlAndCheck(xhat, setpoint, -0.753);

        xhat = updateAndCheck(xhat, u, 0.019, 0.019, 0.028);
        u = controlAndCheck(xhat, setpoint, -0.576);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.017);
        u = controlAndCheck(xhat, setpoint, -0.521);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.006);
        u = controlAndCheck(xhat, setpoint, -0.224);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.001);
        u = controlAndCheck(xhat, setpoint, -0.065);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.0001);
        u = controlAndCheck(xhat, setpoint, -0.011);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.0001);
        u = controlAndCheck(xhat, setpoint, 0.001);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0);
        u = controlAndCheck(xhat, setpoint, 0.002);
    }

    @Test
    public void testNearPiWithoutWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is pi - 0.03
        // so delta is +0.02, should push positive

        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(Math.PI - 0.03, 0), p);

        // initially, state estimate: at zero, motionless
        assertEquals(3.112, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        RandomVector<N2> x = pointEstimator.stateForMeasurementWithZeroU(system.position(Math.PI - 0.03));
        xhat = pooling.fuse(x, xhat);

        assertEquals(3.112, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);

        Matrix<N1, N1> u;
        u = controlAndCheck(xhat, setpoint, 11.455);

        xhat = updateAndCheck(xhat, u, 3.114, 3.114, 0.229);
        u = controlAndCheck(xhat, setpoint, -0.058);

        xhat = updateAndCheck(xhat, u, 3.118, 3.118, 0.228);
        u = controlAndCheck(xhat, setpoint, -2.454);

        xhat = updateAndCheck(xhat, u, 3.122, 3.122, 0.178);
        u = controlAndCheck(xhat, setpoint, -2.517);

        xhat = updateAndCheck(xhat, u, 3.125, 3.125, 0.128);
        u = controlAndCheck(xhat, setpoint, -1.982);

        xhat = updateAndCheck(xhat, u, 3.128, 3.128, 0.089);
        u = controlAndCheck(xhat, setpoint, -1.679);

        xhat = updateAndCheck(xhat, u, 3.130, 3.130, 0.056);
        u = controlAndCheck(xhat, setpoint, -1.279);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.030);
        u = controlAndCheck(xhat, setpoint, -0.806);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.014);
        u = controlAndCheck(xhat, setpoint, -0.302);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.007);
        u = controlAndCheck(xhat, setpoint, -0.076);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.007);
        u = controlAndCheck(xhat, setpoint, -0.008);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.005);
        u = controlAndCheck(xhat, setpoint, 0.004);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.005);
        u = controlAndCheck(xhat, setpoint, 0.003);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.006);
        u = controlAndCheck(xhat, setpoint, 0.001);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.006);
        u = controlAndCheck(xhat, setpoint, 0.001);
    }

    @Test
    public void testNearPiWithWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        // starting point is the only difference
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);

        // initially, state estimate: at zero, motionless
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // starting point is the only difference

        RandomVector<N2> x = pointEstimator.stateForMeasurementWithZeroU(system.position(-1.0 * Math.PI + 0.01));
        xhat = pooling.fuse(x, xhat);

        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);

        Matrix<N1, N1> u;
        u = controlAndCheck(xhat, setpoint, -11.455);

        xhat = updateAndCheck(xhat, u, -3.133, -3.133, -0.229);
        u = controlAndCheck(xhat, setpoint, -0.312);

        xhat = updateAndCheck(xhat, u, -3.138, -3.138, -0.235);
        u = controlAndCheck(xhat, setpoint, 2.638);

        xhat = updateAndCheck(xhat, u, 3.141, 3.141, -0.182);
        u = controlAndCheck(xhat, setpoint, 2.700);

        xhat = updateAndCheck(xhat, u, 3.138, 3.138, -0.128);
        u = controlAndCheck(xhat, setpoint, 2.058);

        xhat = updateAndCheck(xhat, u, 3.135, 3.135, -0.087);
        u = controlAndCheck(xhat, setpoint, 1.700);

        xhat = updateAndCheck(xhat, u, 3.134, 3.134, -0.053);
        u = controlAndCheck(xhat, setpoint, 0.994);

        xhat = updateAndCheck(xhat, u, 3.133, 3.133, -0.033);
        u = controlAndCheck(xhat, setpoint, 0.646);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.021);
        u = controlAndCheck(xhat, setpoint, 0.532);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.01);
        u = controlAndCheck(xhat, setpoint, 0.222);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.005);
        u = controlAndCheck(xhat, setpoint, 0.063);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.005);
        u = controlAndCheck(xhat, setpoint, 0.01);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.004);
        u = controlAndCheck(xhat, setpoint, -0.002);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.004);
        u = controlAndCheck(xhat, setpoint, -0.002);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.004);
        u = controlAndCheck(xhat, setpoint, -0.001);
    }
}
