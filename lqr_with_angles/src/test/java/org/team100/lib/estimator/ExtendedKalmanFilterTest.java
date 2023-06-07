package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.NumericalJacobian;

/**
 * Verifying ExtendedKalmanFilter behavior.
 */
public class ExtendedKalmanFilterTest {
    private static final double kDelta = 0.001;

    public Matrix<N2, N1> f(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u;
        return VecBuilder.fill(pdot, vdot);
    }

    public Matrix<N2, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return x;
    }

    /** Same test as in BitemporalEstimator, to see if it's the same. */
    @Test
    public void testFullCorrection() {
        // System.out.println("FULL EKF");
        Matrix<N2, N1> stateStdDevs = VecBuilder.fill(0.015, 0.17);
        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(0.01, 0.1);
        ExtendedKalmanFilter<N2, N1, N2> estimator = new ExtendedKalmanFilter<>(
                Nat.N2(), Nat.N1(), Nat.N2(), this::f, this::h, stateStdDevs, measurementStdDevs, 0.01);

        // double validTime = 0;
        Matrix<N2, N1> xhat = estimator.getXhat();
        // Matrix<N2, N2> P = estimator.getP();
        // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
        // validTime, xhat.get(0, 0),
        // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));

        for (long i = 10; i < 1000; i += 10) {
            // validTime = 0.001 * i;
            estimator.correct(VecBuilder.fill(0), VecBuilder.fill(1, 0));
            xhat = estimator.getXhat();
            // P = estimator.getP();
            // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
            // validTime, xhat.get(0, 0),
            // xhat.get(1, 0), P.get(0, 0), P.get(0, 1), P.get(1, 0), P.get(1, 1));
        }

        estimator.correct(VecBuilder.fill(0), VecBuilder.fill(1, 0));
        xhat = estimator.getXhat();
        assertEquals(0.814, xhat.get(0, 0), kDelta);
        assertEquals(1.435, xhat.get(1, 0), kDelta);
    }

    /** trying to figure out how to handle the discretization time */
    // turn this off because it makes a lot of output.
    // @Test
    public void testP() {
        printStuff(0.01);
        printStuff(0.1);
    }

    private void printStuff(double dtSeconds) {
        System.out.println("=======================================");
        System.out.println("dt " + dtSeconds);
        Matrix<N2, N1> stateStdDevs = VecBuilder.fill(0.015, 0.17);
        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(0.1, 0.1);

        // this is from the EKF ctor
        Matrix<N2, N2> m_contQ = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stateStdDevs);
        Matrix<N2, N2> m_contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), measurementStdDevs);
        Matrix<N2, N1> m_xHat = new Matrix<>(Nat.N2(), Nat.N1());

        Matrix<N2, N2> contA = NumericalJacobian.numericalJacobianX(
                Nat.N2(), Nat.N2(), this::f, m_xHat, new Matrix<>(Nat.N1(), Nat.N1()));
        System.out.println("contA " + contA);
        // double integrator A
        assertArrayEquals(new double[] { 0, 1, 0, 0 }, contA.getData());

        Matrix<N2, N2> C = NumericalJacobian.numericalJacobianX(
                Nat.N2(), Nat.N2(), this::h, m_xHat, new Matrix<>(Nat.N1(), Nat.N1()));
        System.out.println("C " + C);
        // C is identity since h is identity.
        assertArrayEquals(new double[] { 1, 0, 0, 1 }, C.getData());

        Pair<Matrix<N2, N2>, Matrix<N2, N2>> discPair = Discretization.discretizeAQTaylor(contA, m_contQ, dtSeconds);
        Matrix<N2, N2> discA = discPair.getFirst();
        System.out.println("discA " + discA);
        Matrix<N2, N2> discQ = discPair.getSecond();
        System.out.println("discQ " + discQ);
        Matrix<N2, N2> discR = Discretization.discretizeR(m_contR, dtSeconds);
        System.out.println("discR " + discR);
        Matrix<N2, N2> m_P = Drake.discreteAlgebraicRiccatiEquation(discA.transpose(), C.transpose(), discQ, discR);
        System.out.println("m_P " + m_P);

        // this is from EKF correct()

        // state prediction covariance
        Matrix<N2, N2> S = C.times(m_P).times(C.transpose()).plus(discR);
        System.out.println("S " + S);
        Matrix<N2, N2> K = S.transpose().solve(C.times(m_P.transpose())).transpose();
        System.out.println("K " + K);

        // output
        Matrix<N2, N1> y = VecBuilder.fill(1, 0);
        // input
        Matrix<N1, N1> u = VecBuilder.fill(0);

        Matrix<N2, N1> expectedMeasurement = h(m_xHat, u);
        System.out.println("expectedMeasurement " + expectedMeasurement);
        assertArrayEquals(new double[] { 0, 0 }, expectedMeasurement.getData());

        Matrix<N2, N1> residual = y.minus(expectedMeasurement);
        assertArrayEquals(new double[] { 1, 0 }, residual.getData());
        System.out.println("residual " + residual);

        Matrix<N2, N1> increment = K.times(residual);
        System.out.println("increment " + increment);

        m_xHat = m_xHat.plus(increment);
        System.out.println("m_xHat " + m_xHat);
        m_P = Matrix.eye(Nat.N2())
                .minus(K.times(C))
                .times(m_P)
                .times(Matrix.eye(Nat.N2()).minus(K.times(C)).transpose())
                .plus(K.times(discR).times(K.transpose()));
        System.out.println("m_P " + m_P);
    }

    /** how does R, Q, and dt affect K? */
    // turn this off because it makes a lot of output.
    // @Test
    public void testKRQDt() {
        System.out.println("KRQDt ======================================");
        System.out.println("     q,      r,     dt, k[0,0]");
        // more dt => more k
        for (double dt = 0.1; dt < 10; dt += 0.1) {
            double r = 1;
            double q = 1;
            Matrix<N2, N2> K = k(q, r, dt);
            System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f\n", q, r, dt, K.get(0, 0));
        }
        for (double q = 0.1; q < 10; q += 0.1) {
            double r = 1;
            double dt = 1;
            Matrix<N2, N2> K = k(q, r, dt);
            System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f\n", q, r, dt, K.get(0, 0));
        }
        for (double r = 0.1; r < 10; r += 0.1) {
            double dt = 1;
            double q = 1;
            Matrix<N2, N2> K = k(q, r, dt);
            System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f\n", q, r, dt, K.get(0, 0));
        }
    }

    private Matrix<N2, N2> k(double q, double r, double dtSeconds) {
        Matrix<N2, N1> stateStdDevs = VecBuilder.fill(q, q);
        Matrix<N2, N2> m_contQ = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stateStdDevs);

        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(r, r);
        Matrix<N2, N2> m_contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), measurementStdDevs);

        Matrix<N2, N1> m_xHat = new Matrix<>(Nat.N2(), Nat.N1());

        Matrix<N2, N2> contA = NumericalJacobian.numericalJacobianX(
                Nat.N2(), Nat.N2(), this::f, m_xHat, new Matrix<>(Nat.N1(), Nat.N1()));

        Matrix<N2, N2> C = NumericalJacobian.numericalJacobianX(
                Nat.N2(), Nat.N2(), this::h, m_xHat, new Matrix<>(Nat.N1(), Nat.N1()));

        Pair<Matrix<N2, N2>, Matrix<N2, N2>> discPair = Discretization.discretizeAQTaylor(contA, m_contQ, dtSeconds);
        Matrix<N2, N2> discA = discPair.getFirst();

        Matrix<N2, N2> discQ = discPair.getSecond();

        Matrix<N2, N2> discR = Discretization.discretizeR(m_contR, dtSeconds);

        Matrix<N2, N2> m_P = Drake.discreteAlgebraicRiccatiEquation(discA.transpose(), C.transpose(), discQ, discR);

        Matrix<N2, N2> S = C.times(m_P).times(C.transpose()).plus(discR);

        Matrix<N2, N2> K = S.transpose().solve(C.times(m_P.transpose())).transpose();

        return K;

    }

}
