package org.team100.lib.simulation;

import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;

import org.team100.lib.controller.FeedbackControl;
import org.team100.lib.controller.GainCalculator;
import org.team100.lib.controller.InversionFeedforward;
import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.estimator.TrendEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.storage.BitemporalBuffer;
import org.team100.lib.storage.EditableHistory;
import org.team100.lib.storage.History;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class RoboRIO {
    private static final boolean debug = false;
    private static final long kUsecPerRioLoop = 20000; // 20 ms per rio loop
    private static final double kSecPerUsec = 1e-6;
    private static final double kSecPerRioLoop = kUsecPerRioLoop * kSecPerUsec;

    private final Scenario m_scenario;
    // measurements are bitemporal so we can notice late-arriving ones
    private final BitemporalBuffer<RandomVector<N2>> m_measurements;
    // we rewrite recent state history as needed.
    private final EditableHistory<RandomVector<N2>> m_estimates;
    // control history is immutable.
    private final History<Double> m_control_history;
    private final ExtrapolatingEstimator<N2, N1, N2> predictor;
    private final PointEstimator<N2, N1, N2> pointEstimator;
    private final TrendEstimator<N2, N1, N2> trendEstimator;
    private final LinearPooling<N2> pooling;
    private final InversionFeedforward<N2, N1, N2> feedforward;
    private final FeedbackControl<N2, N1, N2> feedback;

    // the last recordTime we've seen from the buffer
    private long recordTime;

    private final AngularRandomVector<N2> initialState;

    public RoboRIO(Scenario scenario, DoubleIntegratorRotary1D system,
            BitemporalBuffer<RandomVector<N2>> measurements) {
        m_scenario = scenario;
        m_measurements = measurements;
        m_estimates = new EditableHistory<>(1000);
        m_control_history = new History<>(1000);
        initialState = makeInitialState(scenario);
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
    }

    /**
     * Run the roborio code if it's the right time.
     * 
     * the goal is to decide what to do for the next quantum, so here we are looking
     * at the future.
     */
    public void step(CompleteState state) {
        if (state.systemTimeMicrosec % kUsecPerRioLoop == 0) {
            if (debug)
                System.out.println("rio step " + state.actualTimeSec());

            replay(state);
            RandomVector<N2> priorState = predictNow(state);
            Matrix<N1, N1> uff = calculateFeedforward(state);
            double actuationTimeSec = state.actualTimeSec() + kSecPerRioLoop;
            RandomVector<N2> predicted = predictFutureUsingFF(state, priorState, uff, actuationTimeSec);
            Matrix<N1, N1> ufb = calculateFeedback(state, actuationTimeSec, predicted);

            state.ffU = uff.get(0, 0);
            state.fbU = ufb.get(0, 0);
            state.totalU = ufb.plus(uff).get(0, 0);

            // the control applies starting now
            m_control_history.put(state.actualTimeSec(), state.totalU);
        }
    }

    /** Predict the future state with only feedforward. */
    private RandomVector<N2> predictFutureUsingFF(CompleteState state, RandomVector<N2> priorState, Matrix<N1, N1> uff,
            double actuationTimeSec) {
        if (debug)
            System.out.println(
                    "integrating from " + state.actualTimeSec() + " to " + actuationTimeSec + " uff "
                            + uff.get(0, 0));
        RandomVector<N2> predicted3 = predictor.predictWithNoise(
                priorState,
                uff,
                kSecPerRioLoop);

        if (debug)
            System.out.println("estimate position at " + actuationTimeSec + " is " + predicted3.x.get(0, 0));
        // this prediction is wrong, it's just for the feedforward.
        // we could make a better prediction but it's not worth the time
        state.predictedPosition = predicted3.x.get(0, 0);
        state.predictedVelocity = predicted3.x.get(1, 0);
        return predicted3;
    }

    /**
     * Use the midpoint feedforward as the control for the future estimate.
     */
    private Matrix<N1, N1> calculateFeedforward(CompleteState state) {
        double ffTimeSec = state.actualTimeSec() + kSecPerRioLoop / 2;
        Vector<N2> ffReference = m_scenario.getR(ffTimeSec);
        Vector<N2> ffRDot = m_scenario.getRDot(ffTimeSec);
        Matrix<N1, N1> uff = feedforward.calculateWithRAndRDot(ffReference, ffRDot);
        return uff;
    }

    /** Predict the state for the current instant. */
    private RandomVector<N2> predictNow(CompleteState state) {
        double now = state.actualTimeSec();
        Entry<Double, RandomVector<N2>> entry = m_estimates.floor(now);
        if (entry == null)
            entry = Map.entry(0.0, initialState);

        double stateTimeS = entry.getKey();
        RandomVector<N2> priorState = entry.getValue();
        if (debug)
            System.out.println("prior state " + priorState.x.get(0, 0));

        Entry<Double, Double> priorUEntry = m_control_history.floor(stateTimeS);
        if (priorUEntry == null)
            priorUEntry = Map.entry(0.0, 0.0);
        double priorU = priorUEntry.getValue();
        // integrate to the current time with the previous u
        double timeToNow = now - stateTimeS;
        priorState = predictor.predictWithNoise(
                priorState,
                VecBuilder.fill(priorU),
                timeToNow);
        return priorState;
    }

    private Matrix<N1, N1> calculateFeedback(CompleteState state, double actuationTimeSec,
            RandomVector<N2> predicted3) {
        // make another reference for the end of the period
        Vector<N2> nextReference = m_scenario.getR(actuationTimeSec);
        state.referencePosition = nextReference.get(0, 0);
        state.referenceVelocity = nextReference.get(1, 0);
        Vector<N2> nextRDot = m_scenario.getRDot(actuationTimeSec);
        state.referenceAcceleration = nextRDot.get(1, 0);
        // now fill the gap between the prediction and the reference

        if (debug)
            System.out.println("reference position at " + actuationTimeSec + " is " + nextReference.get(0, 0));

        Matrix<N2, N1> error = nextReference.minus(predicted3.x);
        state.errorPosition = error.get(0, 0);
        state.errorVelocity = error.get(1, 0);
        if (debug)
            System.out.println("error position " + error.get(0, 0));

        // correct for disturbance between reference and estimate
        Matrix<N1, N1> ufb = feedback.calculate(predicted3, nextReference);
        if (debug)
            System.out.println("calculate ufb " + ufb.get(0, 0));

        return ufb;

    }

    /** Update the state history with any measurements that are pending. */
    private void replay(CompleteState state) {
        double earliestMeasurementSec = m_measurements.earliestValidTimeForRecordsAfter(recordTime);
        recordTime = state.systemTimeMicrosec;

        // we need to replay all the measurements since then
        NavigableMap<Double, Entry<Long, RandomVector<N2>>> todo = m_measurements
                .validTailMap(earliestMeasurementSec);

        // we don't need the old estimates, we're going to redo them all
        m_estimates.trim(earliestMeasurementSec);

        // loop through the measurements to replay, in valid-time order.
        state.replayCount = 0;
        if (debug)
            System.out.println("replay " + todo.size());
        for (Entry<Double, Entry<Long, RandomVector<N2>>> measurementEntry : todo.entrySet()) {
            state.replayCount += 1;
            // find the time of the measurement
            double measurementTime = measurementEntry.getKey();
            // find the most-recent state earlier than the measurement
            Entry<Double, RandomVector<N2>> entry = m_estimates.floor(measurementTime);
            if (entry == null) {
                entry = Map.entry(0.0, initialState);
            }
            double stateTimeS = entry.getKey();
            RandomVector<N2> priorState = entry.getValue();
            if (debug)
                System.out.println("found position " + priorState.x.get(0, 0));

            // this is the control in use at the time of the prior state
            Entry<Double, Double> historicalUEntry = m_control_history.floor(stateTimeS);
            if (historicalUEntry == null)
                historicalUEntry = Map.entry(0.0, 0.0);
            Double historical_u = historicalUEntry.getValue();
            if (debug)
                System.out.println("found u " + historical_u);

            // there could be multiple controls between the state and the measurement.
            // so make a new state for each control
            NavigableMap<Double, Double> uEntries = m_control_history.validSubMap(stateTimeS, measurementTime);
            for (Entry<Double, Double> uEntry : uEntries.entrySet()) {
                // integrate the prior state and the prior u up to the new u
                double endS = uEntry.getKey();
                double integrationSpanS = endS - stateTimeS;
                if (debug)
                    System.out
                            .println("tween integrating from " + stateTimeS + " to " + endS + " u " + historical_u);
                if (debug)
                    System.out.println("tween prior state " + priorState.x.get(0, 0));
                priorState = predictor.predictWithNoise(
                        priorState,
                        VecBuilder.fill(historical_u),
                        integrationSpanS);
                if (debug)
                    System.out.println("tween estimate position " + priorState.x.get(0, 0));

                historical_u = uEntry.getValue();
                stateTimeS = endS;
            }
            // now the prior state and state time are up to the most-recent change in u
            // and the historical u is also the latest u
            // so integrate the last little bit
            if (debug)
                System.out.println(
                        "replay integrating from " + stateTimeS + " to " + measurementTime + " u " + historical_u);
            if (debug)
                System.out.println("replay prior state " + priorState.x.get(0, 0));
            double stateToMeasurementS = measurementTime - stateTimeS;
            RandomVector<N2> predictedState = predictor.predictWithNoise(
                    priorState,
                    VecBuilder.fill(historical_u),
                    stateToMeasurementS);
            if (debug)
                System.out.println("replay estimate " + predictedState.x.get(0, 0));

            // this is the measurement state
            RandomVector<N2> measurementState = pointEstimator.stateForMeasurementWithZeroU(measurementEntry.getValue().getValue());
            if (debug)
                System.out.println("replay measurement " + measurementState.x.get(0, 0));
            // pool the measurement and the extrapolation
            RandomVector<N2> fused = pooling.fuse(predictedState, measurementState);
            // record the new estimate
            if (debug)
                System.out.println("replay fused " + fused.x.get(0, 0));
            m_estimates.put(measurementTime, fused);
        }
    }

    private AngularRandomVector<N2> makeInitialState(Scenario scenario) {
        // high variance
        Matrix<N2, N2> initP = new Matrix<>(Nat.N2(), Nat.N2());
        initP.set(0, 0, 1e9);
        initP.set(1, 1, 1e9);
        Vector<N2> initx = VecBuilder.fill(scenario.position(0), scenario.velocity(0));
        return new AngularRandomVector<N2>(initx, new Variance<>(initP));
    }
}
