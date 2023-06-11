package org.team100.lib.simulation;

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
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public abstract class Scenario {
    private static final long kUsecPerSimLoop = 2000; // 2 ms per simulation loop
    private static final long kUsecPerRioLoop = 20000; // 20 ms per rio loop
    private static final double kSecPerUsec = 1e-6;
    private static final double kSecPerRioLoop = kUsecPerRioLoop * kSecPerUsec;

    private final CompleteState state;
    private final ExtrapolatingEstimator<N2, N1, N2> predictor;
    private final PointEstimator<N2, N1, N2> pointEstimator;
    private final TrendEstimator<N2, N1, N2> trendEstimator;
    private final LinearPooling<N2> pooling;
    private final InversionFeedforward<N2, N1, N2> feedforward;
    private final FeedbackControl<N2, N1, N2> feedback;
    private final DoubleIntegratorRotary1D system;

    public Scenario() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        system = new DoubleIntegratorRotary1D(w,v);
        state = new CompleteState();
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
        System.out.println("\n\n" + label());
        System.out.println(state.header());
    }

    public void init() {
        state.init(position(0), velocity(0), acceleration(0));
    }

    public void execute() {
        Matrix<N2, N1> x = VecBuilder.fill(0, 0);
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 0.1);
        p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(x, p);

        for (long step = 0; step < 2000; ++step) {
            state.systemTimeMicrosec = step * kUsecPerSimLoop;
            updateActual();
            updateObservation();
            xhat = rioStep(xhat);
            updatePrediction(xhat);
            updateResidual();
            System.out.println(state.toString());
        }
    }

    /**
     * Run the roborio code if it's the right time.
     * 
     * the goal is to decide what to do for the next quantum, so here we are looking
     * at the future.
     * 
     * TODO: add measurement delay
     */
    RandomVector<N2> rioStep(RandomVector<N2> xhat) {
        if (state.systemTimeMicrosec % kUsecPerRioLoop == 0) {
            RandomVector<N2> x;
            // TODO: predict to the current instant
            // or whatever instant these measurements refer to?
            x = pointEstimator.stateForMeasurementWithZeroU(system.position(state.observedPosition));
            xhat = pooling.fuse(x, xhat);
            x = pointEstimator.stateForMeasurementWithZeroU(system.velocity(state.observedVelocity));
            xhat = pooling.fuse(x, xhat);
            // now we have a good estimate for whatever instant the measurements knew about
            // next predict ahead to the actuator target instant

            xhat = predictor.predictWithNoise(xhat, VecBuilder.fill(state.controlU), kSecPerRioLoop);

            // the actual time we want this control to have effect
            // TODO: make this dt/2 for midpoint actuation
            double actuationTimeSec = state.actualTimeSec() + kSecPerRioLoop;
            Vector<N2> nextReference = getR(actuationTimeSec);
            Vector<N2> nextRDot = getRDot(actuationTimeSec);

            // drive the expected system dynamics
            Matrix<N1, N1> uff = feedforward.calculateWithRAndRDot(nextReference, nextRDot);
 
            // correct for disturbance
            Matrix<N1, N1> u = feedback.calculate(xhat, nextReference);

            // actuate
            state.controlU = u.plus(uff).get(0, 0);
        }
        return xhat;
    }

    abstract String label();

    abstract double position(double timeSec);

    abstract double velocity(double timeSec);

    abstract double acceleration(double timeSec);

    /**
     * Update the actual state of the physical system.
     */
    void updateActual() {
        state.actualPosition = position(state.actualTimeSec());
        state.actualVelocity = velocity(state.actualTimeSec());
        state.actualAcceleration = acceleration(state.actualTimeSec());
    }

    /**
     * For now, observations are perfect and instantaneous.
     * 
     * TODO: add lag and noise
     */
    void updateObservation() {
        state.observedPosition = state.actualPosition;
        state.observedVelocity = state.actualVelocity;
        state.observedAcceleration = state.actualAcceleration;
    }

    /**
     * Specify the desired future state, could be from a trajectory
     */
    Vector<N2> getR(double futureTimeSec) {
        double referencePosition = position(futureTimeSec);
        double referenceVelocity = velocity(futureTimeSec);
        return VecBuilder.fill(referencePosition, referenceVelocity);
    }

    /**
     * analytical future state derivative, could be from a trajectory
     */
    Vector<N2> getRDot(double futureTimeSec) {
        double referenceVelocity = velocity(futureTimeSec);
        double referenceAcceleration = acceleration(futureTimeSec);
        return VecBuilder.fill(referenceVelocity, referenceAcceleration);
    }

    void updatePrediction(RandomVector<N2> xhat) {
        state.predictedPosition = xhat.x.get(0, 0);
        state.predictedVelocity = xhat.x.get(1, 0);
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