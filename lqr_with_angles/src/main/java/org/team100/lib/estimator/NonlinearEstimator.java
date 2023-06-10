package org.team100.lib.estimator;

import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * State estimation for nonlinear models.
 */
public class NonlinearEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final IntegratingPredictor<States, Inputs, Outputs> m_predictor;
    private final PointEstimator<States, Inputs, Outputs> m_pointEstimator;
    private final LinearPooling<States> m_pooling;

    /**
     * @param plant   system dynamics, must be control-affine
     * @param pooling fuses state estimates
     */
    public NonlinearEstimator(
            IntegratingPredictor<States, Inputs, Outputs> predictor,
            PointEstimator<States, Inputs, Outputs> pointEstimator,
            LinearPooling<States> pooling) {
        m_predictor = predictor;
        m_pointEstimator = pointEstimator;
        m_pooling = pooling;
    }


    /**
     * Update with specified measurement and zero u (because u doesn't affect state
     * updates)
     */
    public RandomVector<States> correct(
            RandomVector<States> initialState,
            RandomVector<Outputs> y,
            Sensor<States, Inputs, Outputs> sensor) {
        RandomVector<States> x = m_pointEstimator.stateForMeasurementWithZeroU(y, sensor::hinv);
        return m_pooling.fuse(x, initialState);
    }
}
