package org.team100.controller;

import java.util.function.BiFunction;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.NumericalJacobian;

/**
 * For testing LQR with angle wrapping.
 * 
 * Has an LQR inside which is pretty much only used to calculate K.
 * 
 * The model for this test is a 1-DOF arm without gravity.
 * 
 * state: (angle, angular velocity) <= the zeroth element here wraps
 * measurement: angle
 * output: torque, i guess?
 * 
 * TODO: make this generic, linearized LQR for nonlinear models.
 */
public class AngleController {
    private static final Matrix<N1, N1> kUZero = VecBuilder.fill(0);
    private Matrix<N1, N1> actualU = VecBuilder.fill(0);
    /** The model dynamics. */
    private final BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> m_f;
    Matrix<N2, N2> m_Q;
    Matrix<N1, N1> m_R;

    /**
     * @param f model dynamics. must be control-affine, which means that B can
     *          depend on x but not u, so we don't have to pass u to calculate.
     */
    public AngleController(
            BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f,
            Vector<N2> qelms,
            Vector<N1> relms,
            double dtSeconds) {
        m_f = f;
        m_Q = StateSpaceUtil.makeCostMatrix(qelms);
        m_R = StateSpaceUtil.makeCostMatrix(relms);
    }

    /**
     * Output is not aware of actuator limits, that needs to be
     * handled by the caller.
     */
    public Matrix<N1, N1> getU() {
        return actualU;
    }

    /**
     * Output is not aware of actuator limits, that needs to be
     * handled by the caller.
     */
    public double getU(int row) {
        return getU().get(row, 0);
    }

    /**
     * exactly from the LQR constructor
     */
    public static Matrix<N1, N2> calculateK(
            Matrix<N2, N2> A,
            Matrix<N2, N1> B,
            Matrix<N2, N2> Q,
            Matrix<N1, N1> R,
            double dtSeconds) {
        var discABPair = Discretization.discretizeAB(A, B, dtSeconds);
        var discA = discABPair.getFirst();
        var discB = discABPair.getSecond();

        if (!StateSpaceUtil.isStabilizable(discA, discB)) {
            var builder = new StringBuilder("The system passed to the LQR is uncontrollable!\n\nA =\n");
            builder.append(discA.getStorage().toString())
                    .append("\nB =\n")
                    .append(discB.getStorage().toString())
                    .append('\n');
            throw new IllegalArgumentException(builder.toString());
        }

        var S = Drake.discreteAlgebraicRiccatiEquation(discA, discB, Q, R);

        // K = (BᵀSB + R)⁻¹BᵀSA
        Matrix<N1, N2> m_K = discB
                .transpose()
                .times(S)
                .times(discB)
                .plus(R)
                .solve(discB.transpose().times(S).times(discA));
        return m_K;
    }

    /**
     * Using f instead of A and B
     */
    public Matrix<N1, N2> calculateK(Matrix<N2, N1> x, Matrix<N1, N1> u, double dtSeconds) {
        Matrix<N2, N2> A = NumericalJacobian.numericalJacobianX(Nat.N2(), Nat.N2(), m_f, x, u);
        Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(Nat.N2(), Nat.N1(), m_f, x, u);

        var discABPair = Discretization.discretizeAB(A, B, dtSeconds);
        var discA = discABPair.getFirst();
        var discB = discABPair.getSecond();

        if (!StateSpaceUtil.isStabilizable(discA, discB)) {
            var builder = new StringBuilder("The system passed to the LQR is uncontrollable!\n\nA =\n");
            builder.append(discA.getStorage().toString())
                    .append("\nB =\n")
                    .append(discB.getStorage().toString())
                    .append('\n');
            throw new IllegalArgumentException(builder.toString());
        }

        var S = Drake.discreteAlgebraicRiccatiEquation(discA, discB, m_Q, m_R);

        // K = (BᵀSB + R)⁻¹BᵀSA
        Matrix<N1, N2> m_K = discB
                .transpose()
                .times(S)
                .times(discB)
                .plus(m_R)
                .solve(discB.transpose().times(S).times(discA));
        return m_K;
    }

    /**
     * return K(r-x)
     * 
     * Also wraps the angle, which is in row zero.
     * 
     * TODO: extract the wrapping function
     */
    public Matrix<N1, N1> calculate(Matrix<N2, N1> x, Matrix<N2, N1> r, double dtSeconds) {
        Matrix<N1, N2> K = calculateK(x, kUZero, dtSeconds);
        Matrix<N2, N1> error = r.minus(x);
        error.set(0, 0, MathUtil.angleModulus(error.get(0, 0)));
        actualU = K.times(error);
        return actualU;
    }
}