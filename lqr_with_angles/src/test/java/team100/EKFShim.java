package team100;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;

/**
 * Subclasses KalmanFilter, wraps ExtendedKalmanFilter.
 */
public class EKFShim<S extends Num, I extends Num, O extends Num>
        extends KalmanFilter<S, I, O> {

    private final ExtendedKalmanFilter<S, I, O> ekf;

    /** None of these arguments are actually used, except the EKF. */
    public EKFShim(
            Nat<S> states,
            Nat<O> outputs,
            LinearSystem<S, I, O> plant,
            Matrix<S, N1> stateStdDevs,
            Matrix<O, N1> measurementStdDevs,
            double dtSeconds,
            ExtendedKalmanFilter<S, I, O> ekf) {
        super(states, outputs, plant, stateStdDevs, measurementStdDevs, dtSeconds);
        this.ekf = ekf;
    }

    @Override
    public void correct(Matrix<I, N1> u, Matrix<O, N1> y) {
        ekf.correct(u,y);
    }

    @Override
    public Matrix<S, O> getK() {
        throw new UnsupportedOperationException("EKF has no K");
    }

    @Override
    public double getK(int row, int col) {
        throw new UnsupportedOperationException("EKF has no K");
    }

    @Override
    public Matrix<S, N1> getXhat() {
        return ekf.getXhat();
    }

    @Override
    public double getXhat(int row) {
        return ekf.getXhat(row);
    }

    @Override
    public void predict(Matrix<I, N1> u, double dtSeconds) {
        ekf.predict(u, dtSeconds);
    }

    @Override
    public void reset() {
        if (ekf != null) ekf.reset();
    }

    @Override
    public void setXhat(Matrix<S, N1> xhat) {
        ekf.setXhat(xhat);
    }

    @Override
    public void setXhat(int row, double value) {
        ekf.setXhat(row, value);
    }
}
