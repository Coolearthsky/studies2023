package org.team100.lib.system;

import org.team100.lib.controller.ConstantGainLinearizedLQR;
import org.team100.lib.controller.LinearizedPlantInversionFeedforward;
import org.team100.lib.estimator.NonlinearEstimator;
import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
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
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;
    private final ConstantGainLinearizedLQR<States, Inputs, Outputs> m_controller;
    private final LinearizedPlantInversionFeedforward<States, Inputs, Outputs> m_feedforward;
    private final NonlinearEstimator<States, Inputs, Outputs> m_estimator;

    /**
     * Constructs a state-space loop with the given controller, feedforward, and
     * observer. By default, the initial reference is all zeros. Users should call
     * reset with the initial system state before enabling the loop.
     *
     * @param plant       The system to control.
     * @param controller  State-space controller.
     * @param feedforward Plant inversion feedforward.
     * @param estimator   State-space estimator.
     */
    public NonlinearSystemLoop(
            NonlinearPlant<States, Inputs, Outputs> plant,
            ConstantGainLinearizedLQR<States, Inputs, Outputs> controller,
            LinearizedPlantInversionFeedforward<States, Inputs, Outputs> feedforward,
            NonlinearEstimator<States, Inputs, Outputs> estimator) {
        m_plant = plant;
        m_controller = controller;
        m_feedforward = feedforward;
        m_estimator = estimator;
    }

    /**
     * Correct the state estimate x-hat using the measurements in y.
     * 
     * TODO: allow time travel, measurement from the past.
     * 
     * @param y      measurement
     * @param sensor provides h, residual, and stdev involved with the measurement
     */
    public <Rows extends Num> RandomVector<States> correct(RandomVector<States> x, Matrix<Rows, N1> y, Sensor<States, Inputs, Rows> sensor) {
        return m_estimator.correct(x, y, sensor);
    }

    /**
     * integrate forward dt.
     * TODO: use absolute time
     */
    public RandomVector<States> predictState(RandomVector<States> initial, Matrix<Inputs, N1> calculatedU, double dtSeconds) {
       return m_estimator.predictState(initial, calculatedU, dtSeconds);
    }

    /**
     * find controller output to get to reference at dt; uses observer xhat.
     * TODO: use absolute time
     */
    public Matrix<Inputs, N1> calculateTotalU(Matrix<States, N1> xhat, Matrix<States, N1> r, Matrix<States, N1> rDot, double dtSeconds) {
        Matrix<Inputs, N1> controllerU = m_controller.calculate(xhat, r);
        Matrix<Inputs, N1> feedforwardU = m_feedforward.calculateWithRAndRDot(r, rDot);
        return m_plant.limit(controllerU.plus(feedforwardU));
    }
}
