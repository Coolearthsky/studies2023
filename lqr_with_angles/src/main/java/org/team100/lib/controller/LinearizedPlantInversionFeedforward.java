package org.team100.lib.controller;

import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalJacobian;

/**
 * Feedforward for a control-affine system, i.e. control response depends only
 * on state:
 * 
 * dx/dt = f(x,u) = f(x) + g(x)u
 * 
 * Feedforward is calculated by linearizing g(x) and inverting the dynamics.
 * 
 * Linearizing all the time might be slow; maybe this should accept only
 * constant control response.
 *
 * This is similar to the WPI version but it's immutable.
 */
public class LinearizedPlantInversionFeedforward<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final Nat<States> m_states;
    private final Nat<Inputs> m_inputs;
    private final Matrix<Inputs, N1> uZero;
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;

    /**
     * @param f the full dynamics f(x,u)
     */
    public LinearizedPlantInversionFeedforward(
            Nat<States> states,
            Nat<Inputs> inputs,
            NonlinearPlant<States, Inputs, Outputs> plant) {
        m_states = states;
        m_inputs = inputs;
        m_plant = plant;
        uZero = new Matrix<>(inputs, Nat.N1());
    }

    /**
     * @return feedforward as linearized and inverted control response
     */
    public Matrix<Inputs, N1> calculateWithRAndRDot(Matrix<States, N1> r, Matrix<States, N1> rDot) {
        Matrix<States, Inputs> B = NumericalJacobian.numericalJacobianU(m_states, m_inputs, m_plant::f, r, uZero);
        return B.solve(rDot.minus(m_plant.f(r, uZero)));
    }
}
