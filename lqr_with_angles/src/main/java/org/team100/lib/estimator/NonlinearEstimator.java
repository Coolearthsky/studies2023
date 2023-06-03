package org.team100.lib.estimator;

import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;

/**
 * State estimation for nonlinear models.
 * 
 * Has an EKF inside.
 */
public class NonlinearEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final Matrix<Inputs, N1> m_uZero;
    private final NonlinearPlant<States, Inputs, Outputs> m_system;
    private final ExtendedKalmanFilter<States, Inputs, Outputs> ekf;

    /**
     * @param system    system dynamics, must be control-affine
     * @param dtSeconds scales (inversely) measurement noise in correction.
     *                  TODO: do this per-correction instead
     */
    public NonlinearEstimator(NonlinearPlant<States, Inputs, Outputs> system, double dtSeconds) {
        m_uZero = new Matrix<>(system.inputs(), Nat.N1());
        m_system = system;
        ekf = new ExtendedKalmanFilter<States, Inputs, Outputs>(
                system.states(),
                system.inputs(),
                system.outputs(),
                system::f,
                system.full()::h,
                system.stdev(),
                system.full().stdev(),
                system.full()::yResidual,
                system::xAdd,
                dtSeconds);
    }

    /**
     * Predict state, wrapping the angle if required.
     * 
     * @param u     total control output
     * @param dtSec time quantum (sec)
     */
    public void predictState(Matrix<Inputs, N1> u, double dtSec) {
        ekf.predict(u, dtSec);
        final Matrix<States, N1> xhat = ekf.getXhat();
        final Matrix<States, N1> xhatNormalized = m_system.xNormalize(xhat);
        ekf.setXhat(xhatNormalized);
    }

    /**
     * Update with specified measurement and zero u (because u doesn't affect state
     * updates)
     */
    public <Rows extends Num> void correct(Matrix<Rows, N1> y, Sensor<States, Inputs, Rows> sensor) {
        Matrix<Rows, Rows> contR = StateSpaceUtil.makeCovarianceMatrix(sensor.rows(), sensor.stdev());
        ekf.correct(
                sensor.rows(),
                m_uZero,
                y,
                sensor::h,
                contR,
                sensor::yResidual,
                m_system::xAdd);
    }

    public void reset() {
        ekf.reset();
    }

    public void setXhat(Matrix<States, N1> xHat) {
        ekf.setXhat(xHat);
    }

    public double getXhat(int row) {
        return ekf.getXhat(row);
    }

    public double getP(int row, int col) {
        return ekf.getP(row, col);
    }

    public Matrix<States, N1> getXhat() {
        return ekf.getXhat();
    }
}
