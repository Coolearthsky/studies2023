package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.BiFunction;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.NumericalJacobian;

/** To understand what EKF is doing. */
public class EKFCorrectionTest {
    private static Matrix<N1, N1> hVelocity(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return VecBuilder.fill(x.get(1, 0));
    }

    /** This is code copied from the WPI EKF class. */
    @Test
    public void testCorrect() {
        // this is velocity correction.

        // u should actually be 1 so try that
        Matrix<N1, N1> u = VecBuilder.fill(1);
        // velocity measurement
        Matrix<N1, N1> y = VecBuilder.fill(0);
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N1, N1>> h = EKFCorrectionTest::hVelocity;
        Matrix<N1, N1> contR = VecBuilder.fill(0.01);
        BiFunction<Matrix<N1, N1>, Matrix<N1, N1>, Matrix<N1, N1>> residualFuncY = AngleStatistics.angleResidual(0);
        BiFunction<Matrix<N2, N1>, Matrix<N2, N1>, Matrix<N2, N1>> addFuncX = AngleStatistics.angleAdd(0);

        Matrix<N2, N1> m_xHat = VecBuilder.fill(0, 0);
        double m_dtSeconds = 0.02;
        Matrix<N2, N2> m_P = Matrix.mat(Nat.N2(), Nat.N2()).fill(0.013, 0.004, 0.004, 0.009);

        // in this case we just measure velocity so C = [0, 1]
        final Matrix<N1, N2> C = NumericalJacobian.numericalJacobianX(Nat.N1(), Nat.N2(), h, m_xHat, u);
        assertEquals(0, C.get(0, 0), 0.001);
        assertEquals(1, C.get(0, 1), 0.001);

        final Matrix<N1, N1> discR = Discretization.discretizeR(contR, m_dtSeconds);
        // this is just R/dt
        assertEquals(0.5, discR.get(0, 0), 0.001);

        final Matrix<N1, N1> S = C.times(m_P).times(C.transpose()).plus(discR);
        assertEquals(0.509, S.get(0, 0), 0.001);

        // We want to put K = PCᵀS⁻¹ into Ax = b form so we can solve it more
        // efficiently.
        //
        // K = PCᵀS⁻¹
        // KS = PCᵀ
        // (KS)ᵀ = (PCᵀ)ᵀ
        // SᵀKᵀ = CPᵀ
        //
        // The solution of Ax = b can be found via x = A.solve(b).
        //
        // Kᵀ = Sᵀ.solve(CPᵀ)
        // K = (Sᵀ.solve(CPᵀ))ᵀ
        final Matrix<N2, N1> K = S.transpose().solve(C.times(m_P.transpose())).transpose();
        // so K = [0.008, 0.018]
        assertEquals(0.008, K.get(0, 0), 0.001);
        assertEquals(0.018, K.get(1, 0), 0.001);

        // x̂ₖ₊₁⁺ = x̂ₖ₊₁⁻ + K(y − h(x̂ₖ₊₁⁻, uₖ₊₁))
        m_xHat = addFuncX.apply(m_xHat, K.times(residualFuncY.apply(y, h.apply(m_xHat, u))));

        // Pₖ₊₁⁺ = (I−Kₖ₊₁C)Pₖ₊₁⁻(I−Kₖ₊₁C)ᵀ + Kₖ₊₁RKₖ₊₁ᵀ
        // Use Joseph form for numerical stability
        m_P = Matrix.eye(Nat.N2())
                .minus(K.times(C))
                .times(m_P)
                .times(Matrix.eye(Nat.N2()).minus(K.times(C)).transpose())
                .plus(K.times(discR).times(K.transpose()));
    }
}
