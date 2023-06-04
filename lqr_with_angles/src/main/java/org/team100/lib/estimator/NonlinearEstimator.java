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
 * 
 * Has an EKF inside.
 */
public class NonlinearEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final Matrix<Inputs, N1> m_uZero;
    private final NonlinearPlant<States, Inputs, Outputs> m_system;
    private final double correctionDtSec;
    private final AperiodicExtendedKalmanFilter<States, Inputs, Outputs> ekf;

    /**
     * @param plant           system dynamics, must be control-affine
     * @param correctionDtSec scales (inversely) measurement noise in correction.
     *                        This is used only for correction, to discretize the
     *                        DARE solution; choose any value you want (e.g. the
     *                        robot loop period), and then tune the state and
     *                        measurement variances around that.
     */
    public NonlinearEstimator(NonlinearPlant<States, Inputs, Outputs> plant, double correctionDtSec) {
        m_uZero = new Matrix<>(plant.inputs(), Nat.N1());
        m_system = plant;
        this.correctionDtSec = correctionDtSec;
        ekf = new AperiodicExtendedKalmanFilter<States, Inputs, Outputs>(
                plant.states(),
                plant.inputs(),
                plant.outputs(),
                plant::f,
                plant.full()::h,
                plant.stdev(),
                plant.full().stdev(),
                plant.full()::yResidual,
                plant::xAdd,
                correctionDtSec);
    }

    /**
     * Predict state under output u for dtSec in the future and normalize.
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

    public Matrix<States, States> getP() {
        return ekf.getP();
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
