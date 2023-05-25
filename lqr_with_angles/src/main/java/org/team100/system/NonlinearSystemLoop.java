package org.team100.system;

import java.util.function.Function;

import org.team100.controller.AngleController;
import org.team100.estimator.ExtendedAngleEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * A copy of WPILib LinearSystemLoop that works with AngleEKF.
 */
public class NonlinearSystemLoop {
    private final AngleController m_controller;
    // private final LinearPlantInversionFeedforward<N2, N1, N2> m_feedforward;
    private final ControlAffinePlantInversionFeedforward<N2,N1> m_feedforward;
    private final ExtendedAngleEstimator m_observer;
    private Matrix<N2, N1> m_nextR;
    private Function<Matrix<N1, N1>, Matrix<N1, N1>> m_clampFunction;

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
            ControlAffinePlantInversionFeedforward<N2,N1> feedforward,
            ExtendedAngleEstimator observer,
            Function<Matrix<N1, N1>, Matrix<N1, N1>> clampFunction) {
        this.m_controller = controller;
        this.m_feedforward = feedforward;
        this.m_observer = observer;
        this.m_clampFunction = clampFunction;

        m_nextR = VecBuilder.fill(0,0);
        reset(m_nextR);
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
     * Set the next reference r.
     *
     * @param nextR Next reference.
     */
    public void setNextR(Matrix<N2, N1> nextR) {
        m_nextR = nextR;
    }

    /**
     * Returns the controller's calculated control input u plus the calculated
     * feedforward u_ff.
     *
     * @return the calculated control input u.
     */
    public Matrix<N1, N1> getU() {
        return m_clampFunction.apply(
                m_controller.getU().plus(
                        m_feedforward.getUff()));
    }

    /**
     * Returns an element of the controller's calculated control input u.
     *
     * @param row Row of u.
     * @return the calculated control input u at the row i.
     */
    public double getU(int row) {
        return getU().get(row, 0);
    }

    /**
     * Zeroes reference r and controller output u. The previous reference of the
     * PlantInversionFeedforward and the initial state estimate of the KalmanFilter
     * are set to the initial state provided.
     *
     * @param initialState The initial state.
     */
    public void reset(Matrix<N2, N1> initialState) {
        m_nextR.fill(0.0);
        //m_controller.reset();
        m_feedforward.reset(initialState);
        m_observer.setXhat(initialState);
    }

    /**
     * Correct the state estimate x-hat using the measurements in y.
     *
     * @param y Measurement
     */
    public void correctAngle(double y) {
        m_observer.correctAngle(getU().get(0, 0), y);
    }

    public void correctVelocity(double y) {
        m_observer.correctVelocity(getU().get(0, 0), y);
    }

    /**
     * Sets new controller output, projects model forward, and runs observer
     * prediction.
     *
     * <p>
     * After calling this, the user should send the elements of u to the actuators.
     *
     * @param dtSeconds Timestep for model update.
     */
    public void predict(double dtSeconds) {
        var u = m_clampFunction.apply(
                m_controller.calculate(m_observer.getXhat(), m_nextR, dtSeconds)
                        .plus(
                                m_feedforward.calculate(m_nextR)));
        m_observer.predictState(u.get(0, 0), dtSeconds);
    }
}
