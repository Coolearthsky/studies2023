package org.team100.lib.system;

import java.util.function.Function;

import org.team100.lib.controller.AngleController;
import org.team100.lib.controller.ImmutableControlAffinePlantInversionFeedforward;
import org.team100.lib.estimator.ExtendedAngleEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * a better version of the loop thing.
 * 
 * this should do two completely asynchronous things
 * 
 * 1. accept measurements, whenever, timestamped.
 * 2. when you want to actuate something at some time, find the oldest
 * measurement you haven't handled, find the state estimate from that time, and
 * replay all the measurements you have, predicting in between. then finally
 * predict one more time to the chosen actuation time. then calculate the total
 * control output (system input) to achieve the refernce.
 */
public class NonlinearSystemLoop {
    private final AngleController m_controller;
    private final ImmutableControlAffinePlantInversionFeedforward<N2, N1> m_feedforward;
    private final ExtendedAngleEstimator m_observer;
    private final Function<Matrix<N1, N1>, Matrix<N1, N1>> m_clampFunction;

    /**
     * Constructs a state-space loop with the given controller, feedforward, and
     * observer. By default,
     * the initial reference is all zeros. Users should call reset with the initial
     * system state
     * before enabling the loop.
     *
     * @param controller    State-space controller.
     * @param feedforward   Plant inversion feedforward.
     * @param observer      State-space observer.
     * @param clampFunction The function used to clamp the input U.
     */
    public NonlinearSystemLoop(
            AngleController controller,
            ImmutableControlAffinePlantInversionFeedforward<N2, N1> feedforward,
            ExtendedAngleEstimator observer,
            Function<Matrix<N1, N1>, Matrix<N1, N1>> clampFunction) {
        this.m_controller = controller;
        this.m_feedforward = feedforward;
        this.m_observer = observer;
        this.m_clampFunction = clampFunction;
        setXhat(VecBuilder.fill(0, 0));
    }

    /**
     * Returns an element of the observer's state estimate x-hat.
     *
     * @param row Row of x-hat.
     * @return the i-th element of the observer's state estimate x-hat.
     */
    public double getXHat(int row) {
        return m_observer.getXhat(row);
    }

    /**
     * Forces the observer estimate to the provided state.
     */
    public void setXhat(Matrix<N2, N1> state) {
        m_observer.setXhat(state);
    }

    /**
     * Correct the state estimate x-hat using the measurements in y.
     * 
     * these should allow time travel, measurement from the past.
     * 
     * also the "U" value here is not useful; the real "h" functions we use never
     * involve a "u" term
     *
     * @param y Measurement
     */
    public void correctAngle(double y) {
        m_observer.correctAngle(y);
    }

    public void correctVelocity(double y) {
        m_observer.correctVelocity(y);
    }

    /**
     * integrate forward dt.
     * TODO: use absolute time
     */
    public void predictState(Matrix<N1, N1> calculatedU, double dtSeconds) {
        m_observer.predictState(calculatedU.get(0, 0), dtSeconds);
    }

    /**
     * find controller output to get to reference at dt; uses observer xhat.
     * TODO: use absolute time
     */
    public Matrix<N1, N1> calculateTotalU(Matrix<N2, N1> r, Matrix<N2, N1> rDot, double dtSeconds) {
        Matrix<N1, N1> controllerU = m_controller.calculate(m_observer.getXhat(), r, dtSeconds);
        Matrix<N1, N1> feedforwardU = m_feedforward.calculateWithRAndRDot(r, rDot);
        return m_clampFunction.apply(controllerU.plus(feedforwardU));
    }
}
