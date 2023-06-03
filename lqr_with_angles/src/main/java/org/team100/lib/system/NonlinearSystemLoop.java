package org.team100.lib.system;

import java.util.function.Function;

import org.team100.lib.controller.LinearizedLQR;
import org.team100.lib.controller.LinearizedPlantInversionFeedforward;
import org.team100.lib.estimator.NonlinearEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

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
public class NonlinearSystemLoop<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final LinearizedLQR<States, Inputs, Outputs> m_controller;
    private final LinearizedPlantInversionFeedforward<States, Inputs, Outputs> m_feedforward;
    private final NonlinearEstimator<States, Inputs, Outputs> m_observer;
    private final Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> m_clampFunction;

    /**
     * Constructs a state-space loop with the given controller, feedforward, and
     * observer. By default, the initial reference is all zeros. Users should call
     * reset with the initial system state before enabling the loop.
     *
     * @param controller    State-space controller.
     * @param feedforward   Plant inversion feedforward.
     * @param observer      State-space observer.
     * @param clampFunction The function used to clamp the input U.
     */
    public NonlinearSystemLoop(
            Nat<States> states,
            LinearizedLQR<States, Inputs, Outputs> controller,
            LinearizedPlantInversionFeedforward<States, Inputs, Outputs> feedforward,
            NonlinearEstimator<States, Inputs, Outputs> observer,
            Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> clampFunction) {
        m_controller = controller;
        m_feedforward = feedforward;
        m_observer = observer;
        m_clampFunction = clampFunction;
        setXhat(new Matrix<>(states, Nat.N1()));
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
    public void setXhat(Matrix<States, N1> state) {
        m_observer.setXhat(state);
    }

    /**
     * Correct the state estimate x-hat using the measurements in y.
     * 
     * TODO: allow time travel, measurement from the past.
     * 
     * @param y      measurement
     * @param sensor provides h, residual, and stdev involved with the measurement
     */
    public <Rows extends Num> void correct(Matrix<Rows, N1> y, Sensor<States, Inputs, Rows> sensor) {
        m_observer.correct(y, sensor);
    }

    /**
     * integrate forward dt.
     * TODO: use absolute time
     */
    public void predictState(Matrix<Inputs, N1> calculatedU, double dtSeconds) {
        m_observer.predictState(calculatedU, dtSeconds);
    }

    /**
     * find controller output to get to reference at dt; uses observer xhat.
     * TODO: use absolute time
     */
    public Matrix<Inputs, N1> calculateTotalU(Matrix<States, N1> r, Matrix<States, N1> rDot, double dtSeconds) {
        Matrix<Inputs, N1> controllerU = m_controller.calculate(m_observer.getXhat(), r, dtSeconds);
        Matrix<Inputs, N1> feedforwardU = m_feedforward.calculateWithRAndRDot(r, rDot);
        return m_clampFunction.apply(controllerU.plus(feedforwardU));
    }
}
