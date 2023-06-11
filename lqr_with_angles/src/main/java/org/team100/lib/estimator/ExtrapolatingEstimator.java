package org.team100.lib.estimator;

import org.team100.lib.math.RandomVector;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Extrapolates previous state to estimate future state using system dynamics.
 */
public class ExtrapolatingEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;

    public ExtrapolatingEstimator(NonlinearPlant<States, Inputs, Outputs> plant) {
        m_plant = plant;
    }

    /** Integrate f forwards and add xi noise. */
    public RandomVector<States> predictWithNoise(
            RandomVector<States> x,
            Matrix<Inputs, N1> u,
            double dtS) {
        return addNoise(predict(x, u, dtS), dtS);
    }

    /**
     * Predict state under output u for dtSec in the future using Runge-Kutta 4.
     * 
     * @param x     initial state
     * @param u     total control output
     * @param dtSec time quantum (sec)
     */
    RandomVector<States> predict(
            RandomVector<States> x,
            Matrix<Inputs, N1> u,
            double dtS) {
        final var h = dtS;
 
        RandomVector<States> k1 = m_plant.f(x, u);
        RandomVector<States> k2 = m_plant.f(x.plus(k1.times(h * 0.5)), u);
        RandomVector<States> k3 = m_plant.f(x.plus(k2.times(h * 0.5)), u);
        RandomVector<States> k4 = m_plant.f(x.plus(k3.times(h)), u);

        return x.plus(k1.plus(k2.times(2.0)).plus(k3.times(2.0)).plus(k4).times(h / 6.0));
    }

    /** Noise integration produces variance of t */
    RandomVector<States> addNoise(RandomVector<States> x, double dtSeconds) {
        Matrix<States, States> noiseVariance = m_plant.xi().P.copy();
        noiseVariance = noiseVariance.times(dtSeconds);
        return x.make(x.x, x.P.plus(noiseVariance));
    }

}
