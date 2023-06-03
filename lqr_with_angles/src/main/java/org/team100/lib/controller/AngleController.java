package org.team100.lib.controller;

import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
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
public class AngleController<States extends Num> {
    /** u value for calculating K, which is required to be u-invariant. */
    private static final Matrix<N1, N1> kUZero = VecBuilder.fill(0);
    private final Nat<States> m_states;
    private final Matrix<States, States> m_Q;
    private final Matrix<N1, N1> m_R;
    private final NonlinearPlant<States, N1, N2> m_plant;

    /**
     * TODO: i think that we can require B to be constant, since all the real
     * systems we use are like that.
     * in any case it should match the feedforward.
     */
    public AngleController(
            Nat<States> states,
            NonlinearPlant<States, N1, N2> plant,
            Vector<States> qelms,
            Vector<N1> relms) {
        m_states = states;
        m_plant = plant;
        m_Q = StateSpaceUtil.makeCostMatrix(qelms);
        m_R = StateSpaceUtil.makeCostMatrix(relms);
    }

    /**
     * Calculate gains by linearizing around (x,u).
     * 
     * @param x         actual state, e.g. xhat
     * @param u         actual control
     * @param dtSeconds how far in the future
     * @return K
     */
    Matrix<N1, States> calculateK(Matrix<States, N1> x, Matrix<N1, N1> u, double dtSeconds) {
        Matrix<States, States> A = NumericalJacobian.numericalJacobianX(m_states, m_states, m_plant::f, x, u);
        Matrix<States, N1> B = NumericalJacobian.numericalJacobianU(m_states, Nat.N1(), m_plant::f, x, u);

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
        Matrix<N1, States> m_K = discB
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
     * @param x         the actual state, xhat from the estimator
     * @param r         the desired reference state from the trajectory
     * @param dtSeconds how far in the future
     * @return the controller u value. if you want to use this later, e.g. for
     *         correction, you need to remember it.
     */
    public Matrix<N1, N1> calculate(
            Matrix<States, N1> x,
            Matrix<States, N1> r,
            double dtSeconds) {
        Matrix<N1, States> K = calculateK(x, kUZero, dtSeconds);
        Matrix<States, N1> error = m_plant.xResidual(r, x);
        return K.times(error);
    }
}