package org.team100.lib.simulation;

import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.storage.BitemporalBuffer;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N2;

public class Loop {
    // sparkmax is 500us between measurements, maybe try that.
    private static final long kUsecPerSimLoop = 2000; // 2 ms per simulation loop

    private final CompleteState state;
    private final DoubleIntegratorRotary1D system;
    // measurements are bitemporal so we can notice late-arriving ones
    private final BitemporalBuffer<RandomVector<N2>> m_measurements;
    private final PositionSensor positionSensor;
    private final VelocitySensor velocitySensor;
    private final RoboRIO roborio;

    private final Scenario m_scenario;

    public Loop(Scenario scenario) {
        m_scenario = scenario;
        state = new CompleteState();
        m_measurements = new BitemporalBuffer<>(1000);
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        system = new DoubleIntegratorRotary1D(w, v);
        positionSensor = new PositionSensor(system, m_measurements);
        velocitySensor = new VelocitySensor(system, m_measurements);
        roborio = new RoboRIO(scenario, system, m_measurements);
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
