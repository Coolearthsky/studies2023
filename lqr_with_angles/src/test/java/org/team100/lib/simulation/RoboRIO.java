package org.team100.lib.simulation;

import java.util.Map.Entry;

import org.team100.lib.controller.FeedbackControl;
import org.team100.lib.controller.GainCalculator;
import org.team100.lib.controller.InversionFeedforward;
import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.estimator.TrendEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.storage.BitemporalBuffer;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class RoboRIO {
    private static final long kUsecPerSimLoop = 2000; // 2 ms per simulation loop
    private static final long kUsecPerRioLoop = 20000; // 20 ms per rio loop
    private static final double kSecPerUsec = 1e-6;
    private static final double kSecPerRioLoop = kUsecPerRioLoop * kSecPerUsec;

    private final DoubleIntegratorRotary1D system;
    private final BitemporalBuffer<RandomVector<N2>> m_stateBuffer;
    private final ExtrapolatingEstimator<N2, N1, N2> predictor;
    private final PointEstimator<N2, N1, N2> pointEstimator;
    private final TrendEstimator<N2, N1, N2> trendEstimator;
    private final LinearPooling<N2> pooling;
    private final InversionFeedforward<N2, N1, N2> feedforward;
    private final FeedbackControl<N2, N1, N2> feedback;

    private final Scenario m_scenario;

    // the last recordTime we've seen from the buffer
    // private long recordTime;

    // private RandomVector<N2> xhat;

    public RoboRIO(Scenario scenario) {
        m_scenario = scenario;

        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        system = new DoubleIntegratorRotary1D(w, v);
        m_stateBuffer = new BitemporalBuffer<>(1000);
        // initial state: no idea
        Matrix<N2, N2> initP = new Matrix<>(Nat.N2(), Nat.N2());
        initP.set(0, 0, 1e9);
        initP.set(1, 1, 1e9);
        Vector<N2> initx = VecBuilder.fill(0, 0);
        m_stateBuffer.put(0l, 0.0, new AngularRandomVector<N2>(initx, new Variance<>(initP)));

        predictor = new ExtrapolatingEstimator<>(system);
        pointEstimator = new PointEstimator<>(system);
        trendEstimator = new TrendEstimator<>(system);
        pooling = new VarianceWeightedLinearPooling<>();
        feedforward = new InversionFeedforward<>(system);
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.01);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        GainCalculator<N2, N1, N2> gc = new GainCalculator<>(system, stateTolerance, controlTolerance,
                kSecPerRioLoop);
        feedback = new FeedbackControl<>(system, gc.getK());

        // // initial value is zero
        // Matrix<N2, N1> x = VecBuilder.fill(0, 0);
        // Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        // xhat = new AngularRandomVector<>(x, p);
    }

    /**
     * Run the roborio code if it's the right time.
     * 
     * the goal is to decide what to do for the next quantum, so here we are looking
     * at the future.
     * 
     * TODO: add measurement delay
     * 
     * TODO: use dt/2 for midpoint actuation
     */
    public void step(CompleteState state) {

        if (state.systemTimeMicrosec % kUsecPerRioLoop == 0) {

            // ok currently this doesn't do *BI* temporal at all, it just looks at valid
            // times.
            // TODO: fix that

            // STEP 1: update the state history with any measurements that are pending

            // TODO: deal with late-arriving measurements using recordTime

            RandomVector<N2> x;
            // sort the measurement updates by time, oldest first.
            // we know that velocity lags the most, so do that first.
            // TODO: a real sort

            // find the latest state prior to the observation time
            double obsTimeS = state.velocityObservationTimeSec;

            Entry<Double, Entry<Long, RandomVector<N2>>> entry = m_stateBuffer.floor(obsTimeS);
            double stateTimeS = entry.getKey();
            RandomVector<N2> priorState = entry.getValue().getValue();
            double stateToMeasurementS = obsTimeS - stateTimeS;

            // roll the prior state to the measurement time
            // TODO fix the "u" value here, is WRONG WRONG WRONG
            RandomVector<N2> predicted1 = predictor.predictWithNoise(priorState, VecBuilder.fill(state.totalU),
                    stateToMeasurementS);

            // now find the state for the measurement at this time
            RandomVector<N2> measured1 = pointEstimator
                    .stateForMeasurementWithZeroU(system.velocity(state.observedVelocity));
            // since these are for the same time we can pool them
            RandomVector<N2> fused1 = pooling.fuse(predicted1, measured1);

            // record the estimate we just made
            m_stateBuffer.put(state.systemTimeMicrosec, obsTimeS, fused1);

            // do it again for the position measurement

            obsTimeS = state.positionObservationTimeSec;
            // this should pull the thing we just put there
            entry = m_stateBuffer.floor(obsTimeS);
            stateTimeS = entry.getKey();
            priorState = entry.getValue().getValue();
            stateToMeasurementS = obsTimeS - stateTimeS;

            RandomVector<N2> predicted2 = predictor.predictWithNoise(priorState, VecBuilder.fill(state.totalU),
                    stateToMeasurementS);
            RandomVector<N2> measured2 = pointEstimator
                    .stateForMeasurementWithZeroU(system.position(state.observedPosition));
            RandomVector<N2> fused2 = pooling.fuse(predicted2, measured2);

            // this is another good estimate so remember it
            m_stateBuffer.put(state.systemTimeMicrosec, obsTimeS, fused2);

            // STEP 2: predict the state for the actuation time we want

            // this should find the thing we just put in there
            // .. or it might find a more-recent valid time but older record time
            // TODO: fix that
            // try midpoint
            double actuationTimeSec = state.actualTimeSec() + kSecPerRioLoop / 2;
            entry = m_stateBuffer.floor(actuationTimeSec);
            stateTimeS = entry.getKey();
            priorState = entry.getValue().getValue();
            double stateToActuationS = actuationTimeSec - stateTimeS;

            RandomVector<N2> predicted3 = predictor.predictWithNoise(priorState, VecBuilder.fill(state.totalU),
                    stateToActuationS);

            // this is the final estimate (in the future), remember it.
            m_stateBuffer.put(state.systemTimeMicrosec, actuationTimeSec, predicted3);

            // where do we want to be at that instant?
            Vector<N2> nextReference = m_scenario.getR(actuationTimeSec);
            state.referencePosition = nextReference.get(0, 0);
            state.referenceVelocity = nextReference.get(1, 0);
            Vector<N2> nextRDot = m_scenario.getRDot(actuationTimeSec);
            state.referenceAcceleration = nextRDot.get(1, 0);

            // drive the expected system dynamics
            Matrix<N1, N1> uff = feedforward.calculateWithRAndRDot(nextReference, nextRDot);

            Matrix<N2, N1> error = nextReference.minus(predicted3.x);
            state.errorPosition = error.get(0, 0);
            state.errorVelocity = error.get(1, 0);
            // correct for disturbance between reference and estimate
            Matrix<N1, N1> u = feedback.calculate(predicted3, nextReference);

            state.ffU = uff.get(0, 0);
            state.fbU = u.get(0, 0);
            // actuate
            state.totalU = u.plus(uff).get(0, 0);

            state.predictedPosition = predicted3.x.get(0, 0);
            state.predictedVelocity = predicted3.x.get(1, 0);
        }
    }
}
