package org.team100.lib.estimator;

import java.util.Map.Entry;

import org.team100.lib.storage.BitemporalBuffer;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Keeps past states and measurements, rolls back and forth.
 * 
 * TODO: if the recent time is close to the current time, just use it.
 * 
 * TODO: differentiate between predictions and corrections?
 * 
 * TODO: consider removing state from the internal estimator, since we force the
 * state every time.
 */
public class BitemporalEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final BitemporalBuffer<Matrix<States, N1>> m_stateBuffer;
    private final NonlinearEstimator<States, Inputs, Outputs> m_estimator;

    public BitemporalEstimator(
            NonlinearPlant<States, Inputs, Outputs> plant,
            BitemporalBuffer<Matrix<States, N1>> stateBuffer,
            NonlinearEstimator<States, Inputs, Outputs> estimator) {
        m_stateBuffer = stateBuffer;
        m_estimator = estimator;
    }

    /**
     * Prediction. Find the most-recent state earlier than the specified
     * valid time, and integrate forward to estimate the state at the valid time.
     * Record the new state and time.
     */
    public Matrix<States, N1> predict(
            Matrix<Inputs, N1> u,
            long recordTimeUSec,
            double validTimeSec) {
        Entry<Double, Entry<Long, Matrix<States, N1>>> floor = floor(validTimeSec);
        m_estimator.setXhat(floor.getValue().getValue());
        m_estimator.predictState(u, validTimeSec - floor.getKey());
        return update(recordTimeUSec, validTimeSec);
    }

    /**
     * Correction. Find the most-recent state earlier than the specified
     * valid time, and use it as the base for correction to estimate the state at
     * the valid time. Record the new state and time.
     */
    public <Rows extends Num> Matrix<States, N1> correct(
            Matrix<Rows, N1> y,
            Sensor<States, Inputs, Rows> sensor,
            long recordTimeUSec,
            double validTimeSec) {
        Entry<Double, Entry<Long, Matrix<States, N1>>> floor = floor(validTimeSec);
        m_estimator.setXhat(floor.getValue().getValue());
        m_estimator.correct(y, sensor);
        return update(recordTimeUSec, validTimeSec);
    }

    /**
     * Find the most-recent state earlier than the specified valid time.
     */
    Entry<Double, Entry<Long, Matrix<States, N1>>> floor(double validTimeSec) {
        if (validTimeSec < 0)
            throw new IllegalArgumentException("Negative time is not allowed: " + validTimeSec);
        Entry<Double, Entry<Long, Matrix<States, N1>>> floor = m_stateBuffer.validFloorEntry(validTimeSec);
        if (floor == null)
            throw new IllegalStateException("No floor key (not initialized?): " + validTimeSec);
        return floor;
    }

    Matrix<States, N1> update(long recordTimeUSec, double validTimeSec) {
        Matrix<States, N1> newState = m_estimator.getXhat();
        m_stateBuffer.put(recordTimeUSec, validTimeSec, newState);
        return newState;
    }

}
