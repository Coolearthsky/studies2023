package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.ConstantGainLinearizedLQR;
import org.team100.lib.estimator.IntegratingPredictor;
import org.team100.lib.estimator.NonlinearEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
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
 * Test combination of estimator and controller
 */
public class EstimatorAndControlTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    // this is position
    private AngularRandomVector<N2> y1(double yd) {

        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, yd); // position

        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.01); // TODO: pass variance somehow
        yP.set(1, 1, 1e9); // velocity gets "don't know" variance
        return new AngularRandomVector<>(yx, yP);

        // return new AngularRandomVector<>(VecBuilder.fill(yd), VecBuilder.fill(0.1));
    }

    private RandomVector<N2> updateAndCheck(
            IntegratingPredictor<N2, N1, N2> predictor,
            NonlinearEstimator<N2, N1, N2> estimator,
            RandomVector<N2> xhat,
            Matrix<N1, N1> u,
            double y,
            Sensor<N2, N1, N2> sensor,
            double x0,
            double x1) {
        xhat = predictor.predict(xhat, u, kDt);
        xhat = estimator.correct(xhat, y1(y), sensor);
        assertEquals(x0, xhat.x.get(0, 0), kDelta);
        assertEquals(x1, xhat.x.get(1, 0), kDelta);
        return xhat;
    }

    private Matrix<N1, N1> controlAndCheck(
            ConstantGainLinearizedLQR<N2, N1, N2> controller,
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
        final Vector<N2> stateTolerance = VecBuilder
                .fill(0.01, // angle (rad)
                        0.2); // velocity (rad/s)
        final Vector<N1> controlTolerance = VecBuilder
                .fill(12.0); // output (volts)

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1,N2> predictor = new IntegratingPredictor<>(system);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(Nat.N1());
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>( predictor, pointEstimator, pooling);
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
                stateTolerance, controlTolerance, kDt);

        // initially, state estimate: at zero, motionless
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(0, 0), p);
        assertEquals(0, xhat.x.get(0, 0));
        assertEquals(0, xhat.x.get(1, 0));

        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);

        Matrix<N1, N1> u = controlAndCheck(controller, xhat, setpoint, 11.455);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.002, system.position(), 0.002, 0.229);
        u = controlAndCheck(controller, xhat, setpoint, 0.137);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.006, system.position(), 0.006, 0.232);
        u = controlAndCheck(controller, xhat, setpoint, -2.390);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.01, system.position(), 0.01, 0.184);
        u = controlAndCheck(controller, xhat, setpoint, -2.529);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.013, system.position(), 0.013, 0.133);
        u = controlAndCheck(controller, xhat, setpoint, -2.001);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.015, system.position(), 0.015, 0.093);
        u = controlAndCheck(controller, xhat, setpoint, -1.400);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.017, system.position(), 0.017, 0.065);
        u = controlAndCheck(controller, xhat, setpoint, -1.127);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.018, system.position(), 0.018, 0.043);
        u = controlAndCheck(controller, xhat, setpoint, -0.753);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.019, system.position(), 0.019, 0.028);
        u = controlAndCheck(controller, xhat, setpoint, -0.576);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.02, system.position(), 0.02, 0.017);
        u = controlAndCheck(controller, xhat, setpoint, -0.521);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.02, system.position(), 0.02, 0.006);
        u = controlAndCheck(controller, xhat, setpoint, -0.224);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.02, system.position(), 0.02, 0.001);
        u = controlAndCheck(controller, xhat, setpoint, -0.065);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.02, system.position(), 0.02, 0.0001);
        u = controlAndCheck(controller, xhat, setpoint, -0.011);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.02, system.position(), 0.02, 0.0001);
        u = controlAndCheck(controller, xhat, setpoint, 0.001);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 0.02, system.position(), 0.02, 0);
        u = controlAndCheck(controller, xhat, setpoint, 0.002);
    }

    @Test
    public void testNearPiWithoutWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is pi - 0.03
        // so delta is +0.02, should push positive

        final Vector<N2> stateTolerance = VecBuilder
                .fill(0.01, // angle (rad)
                        0.2); // velocity (rad/s)
        final Vector<N1> controlTolerance = VecBuilder
                .fill(12.0); // output (volts)

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1,N2> predictor = new IntegratingPredictor<>(system);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(Nat.N1());
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>( predictor, pointEstimator, pooling);
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
                stateTolerance, controlTolerance, kDt);

        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(Math.PI - 0.03, 0), p);

        // initially, state estimate: at zero, motionless
        assertEquals(3.112, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        xhat = estimator.correct(xhat, y1(Math.PI - 0.03), system.position());
        assertEquals(3.112, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);

        Matrix<N1, N1> u;
        u = controlAndCheck(controller, xhat, setpoint, 11.455);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.114, system.position(), 3.114, 0.229);
        u = controlAndCheck(controller, xhat, setpoint, -0.058);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.118, system.position(), 3.118, 0.228);
        u = controlAndCheck(controller, xhat, setpoint, -2.454);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.122, system.position(), 3.122, 0.178);
        u = controlAndCheck(controller, xhat, setpoint, -2.517);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.125, system.position(), 3.125, 0.128);
        u = controlAndCheck(controller, xhat, setpoint, -1.982);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.128, system.position(), 3.128, 0.089);
        u = controlAndCheck(controller, xhat, setpoint, -1.679);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.130, system.position(), 3.130, 0.056);
        u = controlAndCheck(controller, xhat, setpoint, -1.279);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.131, system.position(), 3.131, 0.030);
        u = controlAndCheck(controller, xhat, setpoint, -0.806);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.131, system.position(), 3.131, 0.014);
        u = controlAndCheck(controller, xhat, setpoint, -0.302);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.131, system.position(), 3.131, 0.007);
        u = controlAndCheck(controller, xhat, setpoint, -0.076);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.131, system.position(), 3.131, 0.007);
        u = controlAndCheck(controller, xhat, setpoint, -0.008);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.131, system.position(), 3.131, 0.005);
        u = controlAndCheck(controller, xhat, setpoint, 0.004);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.131, system.position(), 3.131, 0.005);
        u = controlAndCheck(controller, xhat, setpoint, 0.003);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.131, system.position(), 3.131, 0.006);
        u = controlAndCheck(controller, xhat, setpoint, 0.001);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.131, system.position(), 3.131, 0.006);
        u = controlAndCheck(controller, xhat, setpoint, 0.001);
    }

    @Test
    public void testNearPiWithWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        final Vector<N2> stateTolerance = VecBuilder
                .fill(0.01, // angle (rad)
                        0.2); // velocity (rad/s)
        final Vector<N1> controlTolerance = VecBuilder
                .fill(12.0); // output (volts)

        DoubleIntegratorRotary1D system = new NormalDoubleIntegratorRotary1D();
        IntegratingPredictor<N2, N1,N2> predictor = new IntegratingPredictor<>(system);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(Nat.N1());
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        NonlinearEstimator<N2, N1, N2> estimator = new NonlinearEstimator<>(predictor, pointEstimator, pooling);
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(system,
                stateTolerance, controlTolerance, kDt);

        // starting point is the only difference
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);

        // initially, state estimate: at zero, motionless
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // starting point is the only difference
        xhat = estimator.correct(xhat, y1(-1.0 * Math.PI + 0.01), system.position());

        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);

        Matrix<N1, N1> u;
        u = controlAndCheck(controller, xhat, setpoint, -11.455);

        xhat = updateAndCheck(predictor, estimator, xhat, u, -3.133, system.position(), -3.133, -0.229);
        u = controlAndCheck(controller, xhat, setpoint, -0.312);

        xhat = updateAndCheck(predictor, estimator, xhat, u, -3.138, system.position(), -3.138, -0.235);
        u = controlAndCheck(controller, xhat, setpoint, 2.638);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.141, system.position(), 3.141, -0.182);
        u = controlAndCheck(controller, xhat, setpoint, 2.700);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.138, system.position(), 3.138, -0.128);
        u = controlAndCheck(controller, xhat, setpoint, 2.058);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.135, system.position(), 3.135, -0.087);
        u = controlAndCheck(controller, xhat, setpoint, 1.700);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.134, system.position(), 3.134, -0.053);
        u = controlAndCheck(controller, xhat, setpoint, 0.994);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.133, system.position(), 3.133, -0.033);
        u = controlAndCheck(controller, xhat, setpoint, 0.646);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.132, system.position(), 3.132, -0.021);
        u = controlAndCheck(controller, xhat, setpoint, 0.532);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.132, system.position(), 3.132, -0.01);
        u = controlAndCheck(controller, xhat, setpoint, 0.222);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.132, system.position(), 3.132, -0.005);
        u = controlAndCheck(controller, xhat, setpoint, 0.063);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.132, system.position(), 3.132, -0.005);
        u = controlAndCheck(controller, xhat, setpoint, 0.01);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.132, system.position(), 3.132, -0.004);
        u = controlAndCheck(controller, xhat, setpoint, -0.002);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.132, system.position(), 3.132, -0.004);
        u = controlAndCheck(controller, xhat, setpoint, -0.002);

        xhat = updateAndCheck(predictor, estimator, xhat, u, 3.132, system.position(), 3.132, -0.004);
        u = controlAndCheck(controller, xhat, setpoint, -0.001);
    }
}
