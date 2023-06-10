package org.team100.lib.estimator;

import java.util.function.BiFunction;

import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * State estimation for nonlinear models.
 */
public class NonlinearEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final Matrix<Inputs, N1> m_uZero;
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;
    private final IntegratingPredictor<States, Inputs> m_predictor;
    private final PointEstimator<States, Inputs, Outputs> m_pointEstimator;
    private final LinearPooling<States> m_pooling;

    /**
     * @param plant   system dynamics, must be control-affine
     * @param pooling fuses state estimates
     */
    public NonlinearEstimator(
            NonlinearPlant<States, Inputs, Outputs> plant,
            IntegratingPredictor<States, Inputs> predictor,
            PointEstimator<States, Inputs, Outputs> pointEstimator,
            LinearPooling<States> pooling) {
        m_uZero = new Matrix<>(plant.inputs(), Nat.N1());
        m_plant = plant;
        m_predictor = predictor;
        m_pointEstimator = pointEstimator;
        m_pooling = pooling;
    }

    /**
     * Predict state under output u for dtSec in the future and normalize.
     * 
     * @param x     xhat
     * @param u     total control output
     * @param dtSec time quantum (sec)
     */
    public RandomVector<States> predictState(
            RandomVector<States> x,
            Matrix<Inputs, N1> u,
            double dtSec) {
        return m_predictor.predict(m_plant::f, x, u, dtSec);
    }

    /**
     * Update with specified measurement and zero u (because u doesn't affect state
     * updates)
     */
    public RandomVector<States> correct(
            RandomVector<States> initialState,
            RandomVector<Outputs> y,
            Sensor<States, Inputs, Outputs> sensor) {
        RandomVector<States> x = m_pointEstimator.stateForMeasurement(m_uZero, y, sensor::hinv);
        return m_pooling.fuse(x, initialState);
    }
}
