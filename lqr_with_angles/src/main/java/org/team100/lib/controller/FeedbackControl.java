package org.team100.lib.controller;

import org.team100.lib.math.RandomVector;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Full state controller using constant gain.
 */
public class FeedbackControl<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;
    private final Matrix<Inputs, States> m_K;

    public FeedbackControl(
            NonlinearPlant<States, Inputs, Outputs> plant,
            Matrix<Inputs, States> K) {
        m_plant = plant;
        m_K = K;
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
    public Matrix<Inputs, N1> calculate(RandomVector<States> x, Matrix<States, N1> r) {
        RandomVector<States> rv = x.make(r, new Matrix<>(m_plant.states(), m_plant.states()));
       //System.out.println("K: " + m_K);
        Matrix<States, N1> residual = rv.minus(x).x;
        //System.out.println("residual: " + residual);
        return m_K.times(residual);
    }
}