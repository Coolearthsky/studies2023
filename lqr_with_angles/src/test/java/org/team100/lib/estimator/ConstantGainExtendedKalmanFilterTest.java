package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.NumericalJacobian;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;

/**
 * This is cut-and-paste from WPILib EKFTest, but with changes to match the
 * class under test.
 */
class ConstantGainExtendedKalmanFilterTest {
    private static RandomVector<N5> getDynamics(final RandomVector<N5> x, final Matrix<N2, N1> u) {
        final var motors = DCMotor.getCIM(2);

        final var gr = 7.08; // Gear ratio
        final var rb = 0.8382 / 2.0; // Wheelbase radius (track width)
        final var r = 0.0746125; // Wheel radius
        final var m = 63.503; // Robot mass
        final var J = 5.6; // Robot moment of inertia

        final var C1 = -Math.pow(gr, 2) * motors.KtNMPerAmp / (motors.KvRadPerSecPerVolt * motors.rOhms * r * r);
        final var C2 = gr * motors.KtNMPerAmp / (motors.rOhms * r);
        final var k1 = 1.0 / m + rb * rb / J;
        final var k2 = 1.0 / m - rb * rb / J;

        final var vl = x.x.get(3, 0);
        final var vr = x.x.get(4, 0);
        final var Vl = u.get(0, 0);
        final var Vr = u.get(1, 0);

        final Matrix<N5, N1> resultX = new Matrix<>(Nat.N5(), Nat.N1());
        final var v = 0.5 * (vl + vr);
        resultX.set(0, 0, v * Math.cos(x.x.get(2, 0)));
        resultX.set(1, 0, v * Math.sin(x.x.get(2, 0)));
        resultX.set(2, 0, (vr - vl) / (2.0 * rb));
        resultX.set(3, 0, k1 * ((C1 * vl) + (C2 * Vl)) + k2 * ((C1 * vr) + (C2 * Vr)));
        resultX.set(4, 0, k2 * ((C1 * vl) + (C2 * Vl)) + k1 * ((C1 * vr) + (C2 * Vr)));
        // TODO: real P treatment
        return new RandomVector<N5>(resultX, x.P);
    }

    @SuppressWarnings("PMD.UnusedFormalParameter")
    private static RandomVector<N3> getLocalMeasurementModel(RandomVector<N5> x, Matrix<N2, N1> u) {
        // TODO: real P treatment
        return new RandomVector<>(VecBuilder.fill(x.x.get(2, 0), x.x.get(3, 0), x.x.get(4, 0)),
                new Matrix<>(Nat.N3(), Nat.N3()));
    }

    @SuppressWarnings("PMD.UnusedFormalParameter")
    private static RandomVector<N5> getGlobalMeasurementModel(RandomVector<N5> x, Matrix<N2, N1> u) {
        // TODO: real P treatment
        return new RandomVector<>(
                VecBuilder.fill(x.x.get(0, 0), x.x.get(1, 0), x.x.get(2, 0), x.x.get(3, 0), x.x.get(4, 0)), x.P);
    }

    @Test
    void testInit() {
        final double dtSeconds = 0.00505;

        assertDoesNotThrow(
                () -> {
                    ConstantGainExtendedKalmanFilter<N5, N2, N3> observer = new ConstantGainExtendedKalmanFilter<>(
                            Nat.N5(),
                            Nat.N2(),
                            Nat.N3(),
                            ConstantGainExtendedKalmanFilterTest::getDynamics,
                            ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModel,
                            VecBuilder.fill(0.5, 0.5, 10.0, 1.0, 1.0),
                            VecBuilder.fill(0.0001, 0.01, 0.01),
                            dtSeconds);

                    Matrix<N2, N1> u = VecBuilder.fill(12.0, 12.0);
                    RandomVector<N5> xhat = new RandomVector<>(
                            new Matrix<>(Nat.N5(), Nat.N1()),
                            new Matrix<>(Nat.N5(), Nat.N5()));
                    xhat = observer.predict(xhat, u, dtSeconds);

                    var localY = getLocalMeasurementModel(xhat, u);
                    Matrix<N3, N3> m_contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(),
                            VecBuilder.fill(0.0001, 0.01, 0.01));
                    xhat = observer.correct(Nat.N3(), xhat, u, localY,
                            ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModel,
                            m_contR, Matrix::minus, Matrix::plus);

                    var globalY = getGlobalMeasurementModel(xhat, u);
                    var R = StateSpaceUtil.makeCostMatrix(VecBuilder.fill(0.01, 0.01, 0.0001, 0.5, 0.5));
                    xhat = observer.correct(
                            Nat.N5(), xhat, u, globalY,
                            ConstantGainExtendedKalmanFilterTest::getGlobalMeasurementModel, R, Matrix::minus,
                            Matrix::plus);
                });
    }

    @Test
    void testConvergence() {
        final double dtSeconds = 0.00505;
        final double rbMeters = 0.8382 / 2.0; // Robot radius

        ConstantGainExtendedKalmanFilter<N5, N2, N3> observer = new ConstantGainExtendedKalmanFilter<>(
                Nat.N5(),
                Nat.N2(),
                Nat.N3(),
                ConstantGainExtendedKalmanFilterTest::getDynamics,
                ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModel,
                VecBuilder.fill(0.5, 0.5, 10.0, 1.0, 1.0),
                VecBuilder.fill(0.001, 0.01, 0.01),
                dtSeconds);

        List<Pose2d> waypoints = Arrays.asList(
                new Pose2d(2.75, 22.521, new Rotation2d()),
                new Pose2d(24.73, 19.68, Rotation2d.fromDegrees(5.846)));
        var trajectory = TrajectoryGenerator.generateTrajectory(waypoints, new TrajectoryConfig(8.8, 0.1));

        Matrix<N5, N1> r = new Matrix<>(Nat.N5(), Nat.N1());

        Matrix<N5, N1> nextR = new Matrix<>(Nat.N5(), Nat.N1());
        Matrix<N2, N1> u = new Matrix<>(Nat.N2(), Nat.N1());

        var B = NumericalJacobian.numericalJacobianU(
                Nat.N5(),
                Nat.N2(),
                ConstantGainExtendedKalmanFilterTest::getDynamics,
                new Matrix<>(Nat.N5(), Nat.N1()),
                u);

        Matrix<N5, N1> xhat = VecBuilder.fill(
                trajectory.getInitialPose().getTranslation().getX(),
                trajectory.getInitialPose().getTranslation().getY(),
                trajectory.getInitialPose().getRotation().getRadians(),
                0.0,
                0.0);

        var groundTruthX = xhat;

        Matrix<N3, N3> m_contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(),
                VecBuilder.fill(0.001, 0.01, 0.01));

        double totalTime = trajectory.getTotalTimeSeconds();
        for (int i = 0; i < (totalTime / dtSeconds); i++) {
            var ref = trajectory.sample(dtSeconds * i);
            double vl = ref.velocityMetersPerSecond * (1 - (ref.curvatureRadPerMeter * rbMeters));
            double vr = ref.velocityMetersPerSecond * (1 + (ref.curvatureRadPerMeter * rbMeters));

            nextR.set(0, 0, ref.poseMeters.getTranslation().getX());
            nextR.set(1, 0, ref.poseMeters.getTranslation().getY());
            nextR.set(2, 0, ref.poseMeters.getRotation().getRadians());
            nextR.set(3, 0, vl);
            nextR.set(4, 0, vr);

            var localY = getLocalMeasurementModel(groundTruthX, u);
            var whiteNoiseStdDevs = VecBuilder.fill(0.0001, 0.5, 0.5);

            xhat = observer.correct(Nat.N3(), xhat, u,
                    localY.plus(StateSpaceUtil.makeWhiteNoiseVector(whiteNoiseStdDevs)),
                    ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModel,
                    m_contR, Matrix::minus, Matrix::plus);

            Matrix<N5, N1> rdot = nextR.minus(r).div(dtSeconds);
            u = new Matrix<>(B.solve(rdot.minus(getDynamics(r, new Matrix<>(Nat.N2(), Nat.N1())))));

            xhat = observer.predict(xhat, u, dtSeconds);

            groundTruthX = NumericalIntegration.rk4(
                    ConstantGainExtendedKalmanFilterTest::getDynamics, groundTruthX, u, dtSeconds);

            r = nextR;
        }

        var localY = getLocalMeasurementModel(xhat, u);
        xhat = observer.correct(Nat.N3(), xhat, u, localY,
                ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModel,
                m_contR, Matrix::minus, Matrix::plus);

        var globalY = getGlobalMeasurementModel(xhat, u);
        var R = StateSpaceUtil.makeCostMatrix(VecBuilder.fill(0.01, 0.01, 0.0001, 0.5, 0.5));
        xhat = observer.correct(Nat.N5(), xhat, u, globalY,
                ConstantGainExtendedKalmanFilterTest::getGlobalMeasurementModel, R,
                Matrix::minus, Matrix::plus);

        var finalPosition = trajectory.sample(trajectory.getTotalTimeSeconds());
        assertEquals(finalPosition.poseMeters.getTranslation().getX(), xhat.get(0, 0), 1.0);
        assertEquals(finalPosition.poseMeters.getTranslation().getY(), xhat.get(1, 0), 1.0);
        assertEquals(finalPosition.poseMeters.getRotation().getRadians(), xhat.get(2, 0), 1.0);
        assertEquals(0.0, xhat.get(3, 0), 1.0);
        assertEquals(0.0, xhat.get(4, 0), 1.0);
    }
}
