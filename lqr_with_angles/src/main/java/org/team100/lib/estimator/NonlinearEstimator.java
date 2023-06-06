package org.team100.lib.estimator;

import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;

/**
 * State estimation for nonlinear models.
 */
public class NonlinearEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final Matrix<Inputs, N1> m_uZero;
    private final NonlinearPlant<States, Inputs, Outputs> m_system;
    private final ConstantGainExtendedKalmanFilter<States, Inputs, Outputs> ekf;

    /**
     * @param plant           system dynamics, must be control-affine
     * @param correctionDtSec scales (inversely) measurement noise in correction.
     *                        This is used to discretize the DARE solution for gain
     *                        calculation; choose any value you want (e.g. the
     *                        robot loop period), and then tune the state and
     *                        measurement variances around that.
     */
    public NonlinearEstimator(NonlinearPlant<States, Inputs, Outputs> plant, double correctionDtSec) {
        m_uZero = new Matrix<>(plant.inputs(), Nat.N1());
        m_system = plant;
        ekf = new ConstantGainExtendedKalmanFilter<States, Inputs, Outputs>(
                plant.states(),
                plant.inputs(),
                plant.outputs(),
                plant::f,
                plant.full()::h,
                plant.stdev(),
                plant.full().stdev(),
                correctionDtSec);
    }

    /**
     * Predict state under output u for dtSec in the future and normalize.
     * 
     * @param initialState xhat
     * @param u     total control output
     * @param dtSec time quantum (sec)
     */
    public Matrix<States, N1> predictState(Matrix<States, N1> initialState, Matrix<Inputs, N1> u, double dtSec) {
        final Matrix<States, N1> xhat = ekf.predict(initialState, u, dtSec);
        final Matrix<States, N1> xhatNormalized = m_system.xNormalize(xhat);
        return xhatNormalized;
    }

    /**
     * Update with specified measurement and zero u (because u doesn't affect state
     * updates)
     */
    public <Rows extends Num> Matrix<States, N1> correct(
            Matrix<States, N1> initialState,
            Matrix<Rows, N1> y,
            Sensor<States, Inputs, Rows> sensor) {
        Matrix<Rows, Rows> contR = StateSpaceUtil.makeCovarianceMatrix(sensor.rows(), sensor.stdev());
        return ekf.correct(
                sensor.rows(),
                initialState,
                m_uZero,
                y,
                sensor::h,
                contR,
                sensor::yResidual,
                m_system::xAdd);
    }

    public Matrix<States, States> getP() {
        return ekf.getP();
    }
}
