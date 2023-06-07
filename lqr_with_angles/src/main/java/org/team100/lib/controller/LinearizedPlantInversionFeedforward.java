package org.team100.lib.controller;

import org.team100.lib.math.Jacobian;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

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
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;
    private final Matrix<Inputs, N1> uZero;

    /**
     * @param f the full dynamics f(x,u)
     */
    public LinearizedPlantInversionFeedforward(NonlinearPlant<States, Inputs, Outputs> plant) {
        m_plant = plant;
        uZero = new Matrix<>(plant.inputs(), Nat.N1());
    }

    /**
     * @return feedforward as linearized and inverted control response
     */
    public Matrix<Inputs, N1> calculateWithRAndRDot(Matrix<States, N1> r, Matrix<States, N1> rDot) {
        // TODO something better with this non-random random vector
        RandomVector<States> rv = new RandomVector<>(r, new Matrix<>(m_plant.states(), m_plant.states()));
        Matrix<States, Inputs> B = Jacobian.numericalJacobianU(m_plant.states(), m_plant.inputs(), m_plant::f,
                rv, uZero);
        return B.solve(rDot.minus(m_plant.f(rv, uZero).x));
    }
}
