package org.team100.lib.estimator;

import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * For testing LQR with angle wrapping.
 * 
 * Has an EKF inside.
 * 
 * The model for this test is a 1-DOF arm without gravity.
 * 
 * state: (angle, angular velocity)
 * measurement: (angle, angular velocity) (note these are never updated
 * together)
 * output: torque, i guess?
 */
public class ExtendedAngleEstimator<States extends Num, Inputs extends Num> {
    private final Matrix<Inputs, N1> m_uZero;
    private final NonlinearPlant<States, Inputs, N2> m_system;
    private final ExtendedKalmanFilter<States, Inputs, N2> ekf;

    /**
     * Measurement variances.
     */
    private final Matrix<N2, N2> contR;
    private final Matrix<N1, N1> RAngle;
    private final Matrix<N1, N1> RVelocity;

    /**
     * @param system             system dynamics, must be control-affine
     * @param stateStdDevs       vector of std deviations of state estimates
     * @param measurementStdDevs vector of std deviations per measurement
     * @param residualFuncY      subtract measurements
     */
    public ExtendedAngleEstimator(
            Nat<States> states,
            Nat<Inputs> inputs,
            NonlinearPlant<States, Inputs, N2> system,
            Matrix<States, N1> stateStdDevs,
            Matrix<N2, N1> measurementStdDevs,
            double dtSeconds) {
        m_uZero = new Matrix<>(inputs, Nat.N1());
        m_system = system;
        ekf = new ExtendedKalmanFilter<States, Inputs, N2>(
                states,
                inputs,
                Nat.N2(),
                system::f,
                system.full()::h,
                stateStdDevs,
                measurementStdDevs,
                system.full()::yResidual,
                system::xAdd,
                dtSeconds);
        contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), measurementStdDevs);
        RAngle = contR.block(Nat.N1(), Nat.N1(), 0, 0);
        RVelocity = contR.block(Nat.N1(), Nat.N1(), 1, 1);
    }

    /**
     * Predict state, wrapping the angle if required.
     * 
     * @param u     total control output
     * @param dtSec time quantum (sec)
     */
    public void predictState(Matrix<Inputs, N1> u, double dtSec) {
        ekf.predict(u, dtSec);
        Matrix<States, N1> xhat = ekf.getXhat();
        xhat.set(0, 0, MathUtil.angleModulus(xhat.get(0, 0)));
        ekf.setXhat(xhat);
    }

    /**
     * Update with specified position and zero u (because u doesn't affect state
     * updates)
     */
    // TODO replace this with a single correct() method
    public void correctAngle(Matrix<N1, N1> y, Sensor<States, Inputs, N1> sensor) {
        ekf.correct(
                Nat.N1(),
                m_uZero,
                y,
                sensor::h,
                RAngle,
                sensor::yResidual,
                m_system::xAdd);
    }

    /**
     * Update with specified velocity and zero u (because u doesn't affect state
     * updates)
     */
    public void correctVelocity(Matrix<N1, N1> y, Sensor<States, Inputs, N1> sensor) {
        ekf.correct(
                Nat.N1(),
                m_uZero,
                y,
                sensor::h,
                RVelocity,
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
