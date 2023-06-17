package org.team100.lib.simulation;

/**
 * this is modeled after the sparkmax
 * observes position continuously.
 * records a measurement every 500us and remembers it.
 * makes another measurement in another 500us and discards the first one.
 * every 20ms whatever measurement is available is copied to the 'can bus' queue
 * which holds it for another 2ms.
 */
public class PositionSensor {
    private static final long measurementPeriodUs = 500;
    private static final long messagePeriodUs = 20000;
    double measurementValue;
    long measurementTimestampUs;
    double messageValue;
    long messageTimestampUs;

    public void step(CompleteState state) {
        long timeUs = state.systemTimeMicrosec;
        double currentTime = state.actualTimeSec();

        if (timeUs > measurementTimestampUs + measurementPeriodUs) {
            // take a new measurement
            measurementValue = state.actualPosition;
            measurementTimestampUs = timeUs;
        }

        if (timeUs > messageTimestampUs + messagePeriodUs) {
            // send a measurement over can bus
            messageValue = measurementValue;
            messageTimestampUs = timeUs;
            // let the rio see the one that we sent
            state.observedPosition = messageValue;
            state.positionObservationTimeSec = currentTime;
        }
    }

}
