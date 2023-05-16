package team100;

import java.util.function.BiFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * For testing LQR with angle wrapping.
 * 
 * The model for this test is a 1-DOF arm without gravity.
 * 
 * state: (angle, angular velocity)
 * measurement: angle
 * output: torque, i guess?
 */
public class AngleEKF extends ExtendedKalmanFilter<N2, N1, N1> {
    /**
     * f is the derivative of state, like A and B
     */
    private static Matrix<N2, N1> f(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        // position derivative is velocity
        // velocity derivative is control
        return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
    }

    /**
     * h is the measurement, like C and D
     */
    private static Matrix<N1, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        // measurement is position (angle)
        return VecBuilder.fill(x.get(0, 0));
    }

    public AngleEKF(
            Matrix<N2, N1> stateStdDevs,
            Matrix<N1, N1> measurementStdDevs,
            double dtSeconds) {
        // the AngleStatistics functions below implement the wrapping.
        super(
                Nat.N2(),
                Nat.N1(),
                Nat.N1(),
                AngleEKF::f,
                AngleEKF::h,
                stateStdDevs,
                measurementStdDevs,
                AngleStatistics.angleResidual(0),
                AngleStatistics.angleAdd(0), // zero'th row is the angle
                dtSeconds);
    }

    /**
     * Wrap the angle, which is in row zero.
     */
    @Override
    public void predict(
            Matrix<N1, N1> u,
            BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f,
            double dtSeconds) {
        super.predict(u, f, dtSeconds);
        Matrix<N2, N1> xhat = getXhat();
        xhat.set(0, 0, MathUtil.angleModulus(xhat.get(0, 0)));
        setXhat(xhat);
    }
}
