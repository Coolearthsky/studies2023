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
    private final LinearPooling<States> m_pooling;

    /**
     * @param plant   system dynamics, must be control-affine
     * @param pooling fuses state estimates
     */
    public NonlinearEstimator(
            NonlinearPlant<States, Inputs, Outputs> plant,
            IntegratingPredictor<States, Inputs> predictor,
            LinearPooling<States> pooling) {
        m_uZero = new Matrix<>(plant.inputs(), Nat.N1());
        m_plant = plant;
        m_predictor = predictor;
        m_pooling = pooling;
    }

    /**
     * Predict state under output u for dtSec in the future and normalize.
     * 
     * @param x xhat
     * @param u            total control output
     * @param dtSec        time quantum (sec)
     */
    public RandomVector<States> predictState(
            RandomVector<States> x,
            Matrix<Inputs, N1> u,
            double dtSec) {
return m_predictor.predict(m_plant::f, x, u, dtSec);

        // final RandomVector<States> xhat = m_predictor.predict(m_plant::f, x, u, dtSec);
        // final RandomVector<States> xhatXNormalized = m_plant.xNormalize(xhat);
        // return xhatXNormalized;
    }

    /**
     * Update with specified measurement and zero u (because u doesn't affect state
     * updates)
     */
    public <Rows extends Num> RandomVector<States> correct(
            RandomVector<States> initialState,
            RandomVector<Rows> y,
            Sensor<States, Inputs, Rows> sensor) {
        RandomVector<States> x = stateForMeasurement(m_uZero, y, sensor::hinv);
        return m_pooling.fuse(x, initialState);
    }

    /**
     * Use the inverse h function to get the state corresponding to the measurement.
     */
    public <Rows extends Num> RandomVector<States> stateForMeasurement(
            Matrix<Inputs, N1> u,
            RandomVector<Rows> y,
            BiFunction<RandomVector<Rows>, Matrix<Inputs, N1>, RandomVector<States>> hinv) {
        RandomVector<States> x = hinv.apply(y, u);
        return x;
    }
}
