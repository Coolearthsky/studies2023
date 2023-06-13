package org.team100.lib.simulation;

import edu.wpi.first.math.MathUtil;

public class Loop {
    // sparkmax is 500us between measurements, maybe try that.
    private static final long kUsecPerSimLoop = 2000; // 2 ms per simulation loop
    private static final long kUsecPerRioLoop = 20000; // 20 ms per rio loop
    private static final double kSecPerUsec = 1e-6;

    private final CompleteState state;
    private final PositionSensor positionSensor;
    private final VelocitySensor velocitySensor;
    private final RoboRIO roborio;

    private final Scenario m_scenario;

    public Loop(Scenario scenario) {
        m_scenario = scenario;
        state = new CompleteState();
        positionSensor = new PositionSensor();  
        velocitySensor = new VelocitySensor();
        roborio = new RoboRIO(scenario);
    }

    public void run() {
        state.init(m_scenario.position(0), m_scenario.velocity(0), m_scenario.acceleration(0));
        System.out.println("\n\n" + m_scenario.label());
        System.out.println(state.header());
        for (long step = 0; step < 2000; ++step) {
            state.systemTimeMicrosec = step * kUsecPerSimLoop; // fpgatime
            updateActual();
            updateObservation();
            positionSensor.step(state);
            velocitySensor.step(state);
            roborio.step(state);
            updateResidual();
            System.out.println(state.toString());
        }
    }

    /**
     * Update the actual state of the physical system.
     */
    void updateActual() {
        state.actualPosition = m_scenario.position(state.actualTimeSec());
        state.actualVelocity = m_scenario.velocity(state.actualTimeSec());
        state.actualAcceleration = m_scenario.acceleration(state.actualTimeSec());
    }

    /**
     * For now, observations are perfect and instantaneous.
     * 
     * TODO: add lag and noise
     */
    void updateObservation() {
        // position sensor does this now
        // state.observedPosition = state.actualPosition;
        // state.observedVelocity = state.actualVelocity;
        state.observedAcceleration = state.actualAcceleration;
    }

    /**
     * Update the residuals.
     * 
     * The residual should be zero at the *next* filter quantum.
     * Negative error means the data is below the prediction i.e. prediction should
     * be more negative.
     */
    void updateResidual() {
        state.residualPosition = MathUtil.angleModulus(state.actualPosition - state.predictedPosition);
        state.residualVelocity = state.actualVelocity - state.predictedVelocity;
    }
}
