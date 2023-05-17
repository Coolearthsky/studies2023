package team100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.AngleStatistics;
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
public class AngleEstimator {
    private final ExtendedKalmanFilter<N2, N1, N2> ekf;

    /**
     * Measurement variances.
     */
    private final Matrix<N1, N1> RAngle;
    private final Matrix<N1, N1> RVelocity;

    /**
     * The derivative of state.
     * 
     * x = (position, velocity)
     * xdot = (velocity, control)
     */
    private static Matrix<N2, N1> f(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
    }

    /**
     * Both measurements: (position, velocity)
     */
    private static Matrix<N2, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return x;
    }

    /**
     * Measures angular position.
     */
    private static Matrix<N1, N1> hPosition(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return VecBuilder.fill(x.get(0, 0));
    }

    /**
     * Measures angular velocity.
     */
    private static Matrix<N1, N1> hVelocity(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return VecBuilder.fill(x.get(1, 0));
    }

    /**
     * @param measurementStdDevs vector of std deviations per measurement
     */
    public AngleEstimator(
            Matrix<N2, N1> stateStdDevs,
            Matrix<N2, N1> measurementStdDevs,
            double dtSeconds) {
        ekf = new ExtendedKalmanFilter<N2, N1, N2>(
                Nat.N2(),
                Nat.N1(),
                Nat.N2(),
                AngleEstimator::f,
                AngleEstimator::h,
                stateStdDevs,
                measurementStdDevs,
                AngleStatistics.angleResidual(0),
                // Matrix::plus,
                AngleStatistics.angleAdd(0),
                dtSeconds);
        Matrix<N2, N2> m_contR = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), measurementStdDevs);
        RAngle = m_contR.block(Nat.N1(), Nat.N1(), 0, 0);
        RVelocity = m_contR.block(Nat.N1(), Nat.N1(), 1, 1);
    }

    /**
     * Predict state, wrapping the angle if required.
     * 
     * @param u  control output
     * @param dt time quantum
     */
    public void predictState(double u, double dt) {
        ekf.predict(VecBuilder.fill(u), AngleEstimator::f, dt);
        Matrix<N2, N1> xhat = ekf.getXhat();
        xhat.set(0, 0, MathUtil.angleModulus(xhat.get(0, 0)));
        ekf.setXhat(xhat);
    }

    /**
     * Correct with a specific measurement, specify the measurement function and the
     * stdev of the measurement.
     * 
     * @param u control output
     * @param y angle measurement
     */
    public void correctAngle(double u, double y) {
        ekf.correct(
                Nat.N1(),
                VecBuilder.fill(u),
                VecBuilder.fill(y),
                AngleEstimator::hPosition,
                RAngle,
                AngleStatistics.angleResidual(0),
                AngleStatistics.angleAdd(0));
    }

    public void correctVelocity(double u, double y) {
        ekf.correct(
                Nat.N1(),
                VecBuilder.fill(u),
                VecBuilder.fill(y),
                AngleEstimator::hVelocity,
                RVelocity,
                Matrix::minus,
                AngleStatistics.angleAdd(0));
    }

    public void reset() {
        ekf.reset();
    }

    public void setXhat(Matrix<N2, N1> xHat) {
        ekf.setXhat(xHat);
    }

    public double getXhat(int row) {
        return ekf.getXhat(row);
    }

    public double getP(int row, int col) {
        return ekf.getP(row, col);
    }

    public Matrix<N2, N1> getXhat() {
        return ekf.getXhat();
    }
}
