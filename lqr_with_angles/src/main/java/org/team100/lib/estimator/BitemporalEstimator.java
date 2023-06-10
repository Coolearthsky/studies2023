package org.team100.lib.estimator;

import java.util.Map.Entry;

import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.math.RandomVector;
import org.team100.lib.storage.BitemporalBuffer;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Keeps past states and measurements.
 * 
 * TODO: if asked to provide prediction across past measurements, divide the
 * prediction into pieces and use the measurements.
 * 
 * TODO: differentiate between predictions and corrections?
 */
public class BitemporalEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final BitemporalBuffer<RandomVector<States>> m_stateBuffer;
    private final IntegratingPredictor<States, Inputs, Outputs> m_predictor;
    private final PointEstimator<States, Inputs, Outputs> m_pointEstimator;
    private final LinearPooling<States> m_pooling;

    public BitemporalEstimator(
            NonlinearPlant<States, Inputs, Outputs> plant,
            BitemporalBuffer<RandomVector<States>> stateBuffer,
            IntegratingPredictor<States, Inputs, Outputs> predictor,
            PointEstimator<States, Inputs, Outputs> pointEstimator,
            LinearPooling<States> pooling) {
        m_stateBuffer = stateBuffer;
        m_predictor = predictor;
        m_pointEstimator = pointEstimator;
        m_pooling = pooling;
    }

    /**
     * Prediction. Find the most-recent state earlier than the specified
     * valid time, and integrate forward to estimate the state at the valid time.
     * Record the new state and time.
     */
    public RandomVector<States> predict(
            Matrix<Inputs, N1> u,
            long recordTimeUSec,
            double validTimeSec) {
        Entry<Double, Entry<Long, RandomVector<States>>> floor = floor(validTimeSec);
        RandomVector<States> xhat = floor.getValue().getValue();
        RandomVector<States> newstate = m_predictor.predict(xhat, u, validTimeSec - floor.getKey());
        return update(newstate, recordTimeUSec, validTimeSec);
    }

    /**
     * Correction. Find the most-recent state earlier than the specified
     * valid time, and use it as the base for correction to estimate the state at
     * the valid time. Record the new state and time.
     */
    public RandomVector<States> correct(
            RandomVector<Outputs> y,
            Sensor<States, Inputs, Outputs> sensor,
            long recordTimeUSec,
            double validTimeSec) {
        Entry<Double, Entry<Long, RandomVector<States>>> floor = floor(validTimeSec);
        RandomVector<States> xhat = floor.getValue().getValue();
        RandomVector<States> x = m_pointEstimator.stateForMeasurementWithZeroU(y, sensor::hinv);
        xhat = m_pooling.fuse(x, xhat);
        //xhat = m_estimator.correct(xhat, y, sensor);
        return update(xhat, recordTimeUSec, validTimeSec);
    }

    /**
     * Find the most-recent state earlier than the specified valid time.
     */
    Entry<Double, Entry<Long, RandomVector<States>>> floor(double validTimeSec) {
        if (validTimeSec < 0)
            throw new IllegalArgumentException("Negative time is not allowed: " + validTimeSec);
        Entry<Double, Entry<Long, RandomVector<States>>> floor = m_stateBuffer.validFloorEntry(validTimeSec);
        if (floor == null)
            throw new IllegalStateException("No floor key (not initialized?): " + validTimeSec);
        return floor;
    }

    RandomVector<States> update(RandomVector<States> newState, long recordTimeUSec, double validTimeSec) {
        m_stateBuffer.put(recordTimeUSec, validTimeSec, newState);
        return newState;
    }

}
