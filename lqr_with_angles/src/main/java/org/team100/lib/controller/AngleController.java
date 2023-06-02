package org.team100.lib.controller;

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
 * Full state controller using K from linearized LQR.
 * 
 * Also implements angle wrapping.
 * 
 * TODO: make this generic, linearized LQR for nonlinear models.
 * 
 * Unlike the WPI controllers this class doesn't have any model state in it
 * (e.g. remembering the last U calculated), because immutability is good.
 * If you want to retain results returned, do it yourself.
 */
public class AngleController {
    /** u value for calculating K, which is required to be u-invariant. */
    private static final Matrix<N1, N1> kUZero = VecBuilder.fill(0);
    private final BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> m_f;
    private final Matrix<N2, N2> m_Q;
    private final Matrix<N1, N1> m_R;

    /**
     * TODO: i think that we can require B to be constant, since all the real
     * systems we use are like that.
     * in any case it should match the feedforward.
     * 
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
     * Calculate gains by linearizing around (x,u).
     */
    Matrix<N1, N2> calculateK(Matrix<N2, N1> x, Matrix<N1, N1> u, double dtSeconds) {
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
     * return K(r-x) using a tangent K.
     * 
     * note that recalculating K all the time might be too slow, maybe only do it if
     * (x,u) are far from the previous.
     * 
     * Output is not aware of actuator limits; clamp the output yourself.
     * 
     * Also wraps the angle, which is in row zero.
     * 
     * TODO: extract the wrapping function
     * 
     * @return the controller u value. if you want to use this later, e.g. for
     *         correction, you need to remember it.
     */
    public Matrix<N1, N1> calculate(Matrix<N2, N1> x, Matrix<N2, N1> r, double dtSeconds) {
        Matrix<N1, N2> K = calculateK(x, kUZero, dtSeconds);
        Matrix<N2, N1> error = r.minus(x);
        error.set(0, 0, MathUtil.angleModulus(error.get(0, 0)));
        return K.times(error);
    }
}