package org.team100.lib.controller;

import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.NumericalJacobian;

/**
 * Full state controller using constant K from LQR linearized at zero.
 * 
 * TODO: maybe just specify K, I don't have much faith in the math here.
 */
public class ConstantGainLinearizedLQR<States extends Num, Inputs extends Num, Outputs extends Num> {
    /** u value for calculating K, which is required to be u-invariant. */
    private final Matrix<Inputs, N1> kUZero;
    private final Matrix<States, States> m_Q;
    private final Matrix<Inputs, Inputs> m_R;
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;
    private final Matrix<Inputs, States> m_K;

    /**
     * TODO: i think that we can require B to be constant, since all the real
     * systems we use are like that. In any case it should match the feedforward.
     * 
     * @param dtSeconds time step for discretization. choose a value that's
     *                  convenient, e.g. the robot loop period.
     */
    public ConstantGainLinearizedLQR(
            NonlinearPlant<States, Inputs, Outputs> plant,
            Vector<States> qelms,
            Vector<Inputs> relms,
            double dtSeconds) {
        kUZero = new Matrix<>(plant.inputs(), Nat.N1());
        m_plant = plant;
        m_Q = StateSpaceUtil.makeCostMatrix(qelms);
        m_R = StateSpaceUtil.makeCostMatrix(relms);
        Matrix<States, N1> x = new Matrix<>(plant.states(), Nat.N1());
        m_K = calculateK(x, kUZero, dtSeconds);
    }

    /**
     * Calculate gains by linearizing around (x,u).
     * 
     * @param x         actual state, e.g. xhat
     * @param u         actual control
     * @param dtSeconds how far in the future
     * @return K
     */
    Matrix<Inputs, States> calculateK(Matrix<States, N1> x, Matrix<Inputs, N1> u, double dtSeconds) {
        Matrix<States, States> A = NumericalJacobian.numericalJacobianX(m_plant.states(), m_plant.states(), m_plant::f,
                x, u);
        Matrix<States, Inputs> B = NumericalJacobian.numericalJacobianU(m_plant.states(), m_plant.inputs(), m_plant::f,
                x, u);

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
        Matrix<Inputs, States> m_K = discB
                .transpose()
                .times(S)
                .times(discB)
                .plus(m_R)
                .solve(discB.transpose().times(S).times(discA));
        return m_K;
    }

    /**
     * Returns control output, K(r-x), using constant K.
     * 
     * Output is not aware of actuator limits; clamp the output yourself.
     * 
     * @param x the actual state, xhat from the estimator
     * @param r the desired reference state from the trajectory
     * @return the controller u value. if you want to use this later, e.g. for
     *         correction, you need to remember it.
     */
    public Matrix<Inputs, N1> calculate(Matrix<States, N1> x, Matrix<States, N1> r) {
        return m_K.times(m_plant.xResidual(r, x));
    }
}