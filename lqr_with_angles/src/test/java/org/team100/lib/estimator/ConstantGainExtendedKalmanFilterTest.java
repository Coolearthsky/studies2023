package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.Integration;
import org.team100.lib.math.Jacobian;
import org.team100.lib.math.RandomVector;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/**
 * This is cut-and-paste from WPILib EKFTest, but with changes to match the
 * class under test.
 * 
 * TODO: just delete this whole file
 */
class ConstantGainExtendedKalmanFilterTest {
    private static final double kDelta = 0.001;
    private static final double kBig = 1e9;

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

    private static RandomVector<N3> getLocalMeasurementModel(RandomVector<N5> x, Matrix<N2, N1> u) {
        return new RandomVector<>(VecBuilder.fill(x.x.get(2, 0), x.x.get(3, 0), x.x.get(4, 0)),
                x.P.block(Nat.N3(), Nat.N3(), 2, 2));
    }

    private static RandomVector<N5> getLocalMeasurementModelInv(RandomVector<N3> y, Matrix<N2, N1> u) {
        Matrix<N5, N1> x = VecBuilder.fill(0, 0, y.x.get(0, 0), y.x.get(1, 0), y.x.get(2, 0));
        Matrix<N5, N5> P = new Matrix<>(Nat.N5(), Nat.N5());
        // upper diagonal block is the "don't know" value
        P.set(0, 0, kBig);
        P.set(1, 1, kBig);
        // the lower diagonal block is from y
        P.assignBlock(2, 2, y.P);
        return new RandomVector<>(x, P);
    }

    private static RandomVector<N5> getGlobalMeasurementModel(RandomVector<N5> x, Matrix<N2, N1> u) {
        return x;
    }

    private static RandomVector<N5> getGlobalMeasurementModelInv(RandomVector<N5> y, Matrix<N2, N1> u) {
        return y;
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
                    xhat.P.set(0, 0, 0.1);
                    xhat.P.set(1, 1, 0.1);
                    xhat.P.set(2, 2, 0.1);
                    xhat.P.set(3, 3, 0.1);
                    xhat.P.set(4, 4, 0.1);
                    xhat = observer.predict(xhat, u, dtSeconds);

                    var localY = getLocalMeasurementModel(xhat, u);
                    Matrix<N3, N3> m_contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(),
                            VecBuilder.fill(0.0001, 0.01, 0.01));
                    xhat = observer.correctNew(Nat.N3(), xhat, u, localY,
                            ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModel,
                            ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModelInv,
                            // m_contR, Matrix::minus, Matrix::plus);
                            m_contR, RandomVector::minus, RandomVector::plus);

                    var globalY = getGlobalMeasurementModel(xhat, u);
                    var R = StateSpaceUtil.makeCostMatrix(VecBuilder.fill(0.01, 0.01, 0.0001, 0.5, 0.5));
                    xhat = observer.correctNew(
                            Nat.N5(), xhat, u, globalY,
                            ConstantGainExtendedKalmanFilterTest::getGlobalMeasurementModel,
                            ConstantGainExtendedKalmanFilterTest::getGlobalMeasurementModelInv,
                            R,
                            // Matrix::minus, Matrix::plus);
                            RandomVector::minus, RandomVector::plus);
                });
    }

    // TODO figure out why this doesn't work
    // @Test
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

        // grr, reference should not be random variable

        Matrix<N5, N1> r = new Matrix<>(Nat.N5(), Nat.N1());

        Matrix<N5, N1> nextR = new Matrix<>(Nat.N5(), Nat.N1());
        Matrix<N2, N1> u = new Matrix<>(Nat.N2(), Nat.N1());

        var B = Jacobian.numericalJacobianU(
                Nat.N5(),
                Nat.N2(),
                ConstantGainExtendedKalmanFilterTest::getDynamics,
                new RandomVector<>(new Matrix<>(Nat.N5(), Nat.N1()), new Matrix<>(Nat.N5(), Nat.N5())),
                u);

        Matrix<N5, N5> p = new Matrix<>(Nat.N5(), Nat.N5());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        p.set(2, 2, 0.1);
        p.set(3, 3, 0.1);
        p.set(4, 4, 0.1);
        RandomVector<N5> xhat = new RandomVector<>(VecBuilder.fill(
                trajectory.getInitialPose().getTranslation().getX(),
                trajectory.getInitialPose().getTranslation().getY(),
                trajectory.getInitialPose().getRotation().getRadians(),
                0.0,
                0.0), p);

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

            xhat = observer.correctNew(Nat.N3(), xhat, u,
                    localY.plus(
                            new RandomVector<>(
                                    StateSpaceUtil.makeWhiteNoiseVector(whiteNoiseStdDevs),
                                    new Matrix<>(Nat.N3(), Nat.N3()))),
                    ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModel,
                    ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModelInv,
                    m_contR, RandomVector::minus, RandomVector::plus);

            Matrix<N5, N1> rdot = nextR.minus(r).div(dtSeconds);
            RandomVector<N5> rv = new RandomVector<>(r, new Matrix<>(Nat.N5(), Nat.N5()));
            u = new Matrix<>(B.solve(rdot.minus(getDynamics(rv, new Matrix<>(Nat.N2(), Nat.N1())).x)));

            xhat = observer.predict(xhat, u, dtSeconds);

            groundTruthX = Integration.rk4(
                    ConstantGainExtendedKalmanFilterTest::getDynamics, groundTruthX, u, dtSeconds);

            r = nextR;
        }

        var localY = getLocalMeasurementModel(xhat, u);
        xhat = observer.correctNew(Nat.N3(), xhat, u, localY,
                ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModel,
                ConstantGainExtendedKalmanFilterTest::getLocalMeasurementModelInv,
                m_contR, RandomVector::minus, RandomVector::plus);

        var globalY = getGlobalMeasurementModel(xhat, u);
        var R = StateSpaceUtil.makeCostMatrix(VecBuilder.fill(0.01, 0.01, 0.0001, 0.5, 0.5));
        xhat = observer.correctNew(Nat.N5(), xhat, u, globalY,
                ConstantGainExtendedKalmanFilterTest::getGlobalMeasurementModel,
                ConstantGainExtendedKalmanFilterTest::getGlobalMeasurementModelInv,
                R,
                RandomVector::minus, RandomVector::plus);

        var finalPosition = trajectory.sample(trajectory.getTotalTimeSeconds());
        assertEquals(finalPosition.poseMeters.getTranslation().getX(), xhat.x.get(0, 0), 1.0);
        assertEquals(finalPosition.poseMeters.getTranslation().getY(), xhat.x.get(1, 0), 1.0);
        assertEquals(finalPosition.poseMeters.getRotation().getRadians(), xhat.x.get(2, 0), 1.0);
        assertEquals(0.0, xhat.x.get(3, 0), 1.0);
        assertEquals(0.0, xhat.x.get(4, 0), 1.0);
    }

    ///////////////////////////

    // maybe don't delete this part

    /** xdot for x */
    public RandomVector<N2> f(RandomVector<N2> x, Matrix<N1, N1> u) {
        return x;
    }

    /** y for x */
    public RandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
        return x;
    }

    /** x for y */
    public RandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
        return y;
    }

    // this is the old way
    @Test
    public void testSimpleCorrections() {
        ConstantGainExtendedKalmanFilter<N2, N1, N2> observer = new ConstantGainExtendedKalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                Nat.N2(),
                this::f,
                this::h,
                VecBuilder.fill(0.1, 0.1),
                VecBuilder.fill(0.1, 0.1),
                0.02);
        // initial xhat is zero
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
        xP.set(0, 0, 0.01);
        xP.set(1, 1, 0.01);
        RandomVector<N2> xhat = new RandomVector<>(xx, xP);

        // measurement is 1,0
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.01);
        yP.set(1, 1, 0.01);
        RandomVector<N2> y = new RandomVector<>(yx, yP);

        Matrix<N2, N2> contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), VecBuilder.fill(0.1, 0.1));

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        xhat = observer.correct(Nat.N2(), xhat, u, y, this::h, contR, RandomVector::minus, RandomVector::plus);
        assertArrayEquals(new double[] { 0.047, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.025, 0, 0, 0.025 }, xhat.P.getData(), kDelta);

        xhat = observer.correct(Nat.N2(), xhat, u, y, this::h, contR, RandomVector::minus, RandomVector::plus);
        assertArrayEquals(new double[] { 0.092, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.025, 0, 0, 0.025 }, xhat.P.getData(), kDelta);

        xhat = observer.correct(Nat.N2(), xhat, u, y, this::h, contR, RandomVector::minus, RandomVector::plus);
        assertArrayEquals(new double[] { 0.134, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.025, 0, 0, 0.025 }, xhat.P.getData(), kDelta);

        xhat = observer.correct(Nat.N2(), xhat, u, y, this::h, contR, RandomVector::minus, RandomVector::plus);
        assertArrayEquals(new double[] { 0.175, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.025, 0, 0, 0.025 }, xhat.P.getData(), kDelta);

        xhat = observer.correct(Nat.N2(), xhat, u, y, this::h, contR, RandomVector::minus, RandomVector::plus);
        assertArrayEquals(new double[] { 0.213, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.025, 0, 0, 0.025 }, xhat.P.getData(), kDelta);

        // go all the way to the end
        for (int i = 0; i < 100; ++i) {
            xhat = observer.correct(Nat.N2(), xhat, u, y, this::h, contR, RandomVector::minus, RandomVector::plus);

        }
        assertArrayEquals(new double[] { 0.994, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.025, 0, 0, 0.025 }, xhat.P.getData(), kDelta);
    }

    @Test
    public void testStateForMeasurement() {
        ConstantGainExtendedKalmanFilter<N2, N1, N2> observer = new ConstantGainExtendedKalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                Nat.N2(),
                this::f,
                this::h,
                VecBuilder.fill(0.1, 0.1),
                VecBuilder.fill(0.1, 0.1),
                0.02);
        // initial xhat is zero
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
        xP.set(0, 0, 0.01);
        xP.set(1, 1, 0.01);
        RandomVector<N2> xhat = new RandomVector<>(xx, xP);

        // measurement is 1,0
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.01);
        yP.set(1, 1, 0.01);
        RandomVector<N2> y = new RandomVector<>(yx, yP);

        Matrix<N2, N2> contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), VecBuilder.fill(0.1, 0.1));

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        xhat = observer.stateForMeasurement(Nat.N2(), xhat, u, y, this::h, this::hinv, contR, RandomVector::minus,
                RandomVector::plus);
        // since the state is just the measurement,
        // you get the specified mean and variance of the measurement.
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.01, 0, 0, 0.01 }, xhat.P.getData(), kDelta);
    }

    /**
     * this method converges *much* faster than the "kalman gain" method, and it
     * seems more correct and less mysterious.
     */
    @Test
    public void testCombineStateForMeasurement() {
        ConstantGainExtendedKalmanFilter<N2, N1, N2> observer = new ConstantGainExtendedKalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                Nat.N2(),
                this::f,
                this::h,
                VecBuilder.fill(0.1, 0.1),
                VecBuilder.fill(0.1, 0.1),
                0.02);
        // initial xhat is zero
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
        xP.set(0, 0, 0.01);
        xP.set(1, 1, 0.01);
        RandomVector<N2> xhat = new RandomVector<>(xx, xP);
        RandomVector<N2> xhatNew = xhat.copy();

        // measurement is 1,0
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, 1);
        Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
        yP.set(0, 0, 0.01);
        yP.set(1, 1, 0.01);
        RandomVector<N2> y = new RandomVector<>(yx, yP);

        Matrix<N2, N2> contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), VecBuilder.fill(0.1, 0.1));

        LinearPooling<N2> p2 = new VarianceWeightedLinearPooling<N2>();

        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        xhatNew = observer.stateForMeasurement(Nat.N2(), xhat, u, y, this::h, this::hinv, contR, RandomVector::minus,
                RandomVector::plus);
        xhat = p2.fuse(xhat, xhatNew);
        // since the old and new have the same variance the mean is in the middle
        assertArrayEquals(new double[] { 0.5, 0 }, xhat.x.getData(), kDelta);
        // the difference in means adds to the variance but only of the first component
        assertArrayEquals(new double[] { 0.26, 0, 0, 0.01 }, xhat.P.getData(), kDelta);

        xhatNew = observer.stateForMeasurement(Nat.N2(), xhat, u, y, this::h, this::hinv, contR, RandomVector::minus,
                RandomVector::plus);
        xhat = p2.fuse(xhat, xhatNew);
        // new measurement has lower variance so it is preferred
        assertArrayEquals(new double[] { 0.982, 0 }, xhat.x.getData(), kDelta);
        // mean dispersion keeps increasing P
        assertArrayEquals(new double[] { 0.028, 0, 0, 0.01 }, xhat.P.getData(), kDelta);

        xhatNew = observer.stateForMeasurement(Nat.N2(), xhat, u, y, this::h, this::hinv, contR, RandomVector::minus,
                RandomVector::plus);
        xhat = p2.fuse(xhat, xhatNew);
        assertArrayEquals(new double[] { 0.995, 0 }, xhat.x.getData(), kDelta);
        // mean dispersion is on the way down now
        assertArrayEquals(new double[] { 0.015, 0, 0, 0.01 }, xhat.P.getData(), kDelta);

        // go all the way to the end
        for (int i = 0; i < 100; ++i) {
            xhatNew = observer.stateForMeasurement(Nat.N2(), xhat, u, y, this::h, this::hinv, contR,
                    RandomVector::minus, RandomVector::plus);
            xhat = p2.fuse(xhat, xhatNew);
        }
        // now the estimate matches the measurement
        assertArrayEquals(new double[] { 1, 0 }, xhat.x.getData(), kDelta);
        assertArrayEquals(new double[] { 0.01, 0, 0, 0.01 }, xhat.P.getData(), kDelta);
    }
}
