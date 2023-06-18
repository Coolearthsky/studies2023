package org.team100.lib.simulation;

import org.team100.lib.math.RandomVector;
import org.team100.lib.storage.BitemporalBuffer;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.numbers.N2;

/**
 * this is modeled after the sparkmax.
 * observes position continuously
 * records a measurement every 500us and remembers the past N, say, 10?
 * uses the front and back of the window to compute velocity as the secant of
 * position.
 * 
 * currently this just does 20ms windows
 * TODO: actually do the correct thing.
 */
public class VelocitySensor {
    private static final long measurementPeriodUs = 500;
    private static final long messagePeriodUs = 20000;

    private final DoubleIntegratorRotary1D system;
    private final BitemporalBuffer<RandomVector<N2>> m_measurements;

    double measurementValue;
    long measurementTimestampUs;
    double messageValue;
    long messageTimestampUs;

    public VelocitySensor(DoubleIntegratorRotary1D system, BitemporalBuffer<RandomVector<N2>> measurements) {
        this.system = system;
        m_measurements = measurements;
    }

    public void step(CompleteState state) {
        long timeUs = state.systemTimeMicrosec;
        double currentTime = state.actualTimeSec();

        if (timeUs > measurementTimestampUs + measurementPeriodUs) {
            // take a new measurement
            measurementValue = state.actualVelocity;
            measurementTimestampUs = timeUs;
        }

        if (timeUs > messageTimestampUs + messagePeriodUs) {
            // send a measurement over can bus
            messageValue = measurementValue;
            messageTimestampUs = timeUs;
            // let the rio see the one that we sent
            state.observedVelocity = messageValue;
            state.velocityObservationTimeSec = currentTime;
            m_measurements.put(timeUs, currentTime, system.velocity(messageValue));

        }
    }

}
