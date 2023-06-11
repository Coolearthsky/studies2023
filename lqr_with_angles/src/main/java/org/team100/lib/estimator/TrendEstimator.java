package org.team100.lib.estimator;

import org.team100.lib.math.RandomVector;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** State from measurement trend. */
public class TrendEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;

    public TrendEstimator(NonlinearPlant<States, Inputs, Outputs> plant) {
        m_plant = plant;
    }

    public RandomVector<States> stateForMeasurementPair(
            Matrix<Inputs, N1> u,
            RandomVector<Outputs> y0,
            RandomVector<Outputs> y1,
            double dtS) {
        RandomVector<States> x0 = m_plant.hinv(y0, u);
        RandomVector<States> x1 = m_plant.hinv(y1, u);
        RandomVector<States> xdot = x1.minus(x0).times(1.0/dtS);
        RandomVector<States> x = m_plant.finvWrtX(xdot, u);
        return x;
    }

}
