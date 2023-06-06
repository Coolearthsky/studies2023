package org.team100.lib.estimator;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.NumericalJacobian;
import java.util.function.BiFunction;

import org.team100.lib.math.Jacobian;
import org.team100.lib.math.RandomVector;

/**
 * Similar to WPILib EKF but without the P update, just use initial P forever,
 * which means the gain is kinda constant (adjusted by the h stdev though).
 * 
 * TODO: bring this up to the github tip version
 * 
 * TODO: maybe just specify K, I don't really believe any of this math.
 */
public class ConstantGainExtendedKalmanFilter<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final Nat<States> m_states;
    private final BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> m_f;
    private final Matrix<States, States> m_contQ;
    private final Matrix<Outputs, Outputs> m_contR;
    private final double m_correctionDtSec;
    private final Matrix<States, States> m_P;

    /**
     * Constructs an extended Kalman filter.
     *
     * @param states             a Nat representing the number of states.
     * @param inputs             a Nat representing the number of inputs.
     * @param outputs            a Nat representing the number of outputs.
     * @param f                  A vector-valued function of x and u that returns
     *                           the derivative of the state vector.
     * @param h                  A vector-valued function of x and u that returns
     *                           the measurement vector.
     * @param stateStdDevs       Standard deviations of model states.
     * @param measurementStdDevs Standard deviations of measurements.
     * @param correctionDtSec    Nominal discretization timestep.
     *                           This is used for correction, to discretize the
     *                           DARE solution, and for prediction, to calculate P;
     *                           choose any value you want (e.g. the robot loop
     *                           period), and then tune the state and measurement
     *                           variances around that.
     */
    public ConstantGainExtendedKalmanFilter(
            Nat<States> states,
            Nat<Inputs> inputs,
            Nat<Outputs> outputs,
            BiFunction<RandomVector<States>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            BiFunction<RandomVector<States>, Matrix<Inputs, N1>, Matrix<Outputs, N1>> h,
            Matrix<States, N1> stateStdDevs,
            Matrix<Outputs, N1> measurementStdDevs,
            double correctionDtSec) {
        m_states = states;
        m_f = f;
        m_contQ = StateSpaceUtil.makeCovarianceMatrix(states, stateStdDevs);
        m_contR = StateSpaceUtil.makeCovarianceMatrix(outputs, measurementStdDevs);
        m_correctionDtSec = correctionDtSec;

        Matrix<States, N1> xHatZero = new Matrix<>(m_states, Nat.N1());
        Matrix<Inputs, N1> uZero = new Matrix<>(inputs, Nat.N1());
        Matrix<States, States> contA = NumericalJacobian.numericalJacobianX(states, states, f, xHatZero, uZero);
        Matrix<Outputs, States> C = NumericalJacobian.numericalJacobianX(outputs, states, h, xHatZero, uZero);
        Pair<Matrix<States, States>, Matrix<States, States>> discPair = Discretization.discretizeAQTaylor(contA,
                m_contQ, correctionDtSec);
        Matrix<States, States> discA = discPair.getFirst();
        Matrix<States, States> discQ = discPair.getSecond();
        Matrix<Outputs, Outputs> discR = Discretization.discretizeR(m_contR, correctionDtSec);

        // TODO: this P logic is all wrong

        Matrix<States, States> m_initP;
        if (StateSpaceUtil.isDetectable(discA, C) && outputs.getNum() <= states.getNum()) {
            m_initP = Drake.discreteAlgebraicRiccatiEquation(discA.transpose(), C.transpose(), discQ, discR);
        } else {
            m_initP = new Matrix<>(states, states);
        }

        // add the stuff the other class did to update P.

        // this is from predict()
        m_initP = discA.times(m_initP).times(discA.transpose()).plus(discQ);

        // this is from correct()
        // try with full output
        Matrix<Outputs, Outputs> S = C.times(m_initP).times(C.transpose()).plus(discR);
        Matrix<States, Outputs> K = S.transpose().solve(C.times(m_initP.transpose())).transpose();

        m_initP = Matrix.eye(m_states)
                .minus(K.times(C))
                .times(m_initP)
                .times(Matrix.eye(m_states).minus(K.times(C)).transpose())
                .plus(K.times(discR).times(K.transpose()));

        m_P = m_initP;
    }

    /**
     * Correct the state estimate using the measurements in y.
     *
     * @param <Rows>        Number of rows in the result of f(x, u).
     * @param rows          Number of rows in the result of f(x, u).
     * @param m_xHat state
     * @param u             Same control input used in the predict step.
     * @param y             Measurement vector.
     * @param h             A vector-valued function of x and u that returns the
     *                      measurement vector.
     * @param contR         Continuous measurement noise covariance matrix. This is
     *                      probably constant but could be usefully adjusted, e.g.
     *                      if a gyro can detect "not moving" state then it can be
     *                      treated as more accurate than in the noisy "moving"
     *                      state.
     * @param residualFuncY A function that computes the residual of two measurement
     *                      vectors (i.e. it
     *                      subtracts them.)
     * @param addFuncX      A function that adds two state vectors.
     */
    public <Rows extends Num> RandomVector<States> correct(
            Nat<Rows> rows,
            RandomVector<States> m_xHat,
            Matrix<Inputs, N1> u,
            Matrix<Rows, N1> y,
            BiFunction<RandomVector<States>, Matrix<Inputs, N1>, Matrix<Rows, N1>> h,
            Matrix<Rows, Rows> contR,
            BiFunction<Matrix<Rows, N1>, Matrix<Rows, N1>, Matrix<Rows, N1>> residualFuncY,
            BiFunction<RandomVector<States>, RandomVector<States>, RandomVector<States>> addFuncX) {
        Matrix<Rows, States> C = Jacobian.numericalJacobianX(rows, m_states, h, m_xHat.x, u);
        Matrix<Rows, Rows> discR = Discretization.discretizeR(contR, m_correctionDtSec);
        Matrix<Rows, Rows> S = C.times(m_P).times(C.transpose()).plus(discR);
        Matrix<States, Rows> K = S.transpose().solve(C.times(m_P.transpose())).transpose();
        Matrix<States, N1> xhat = addFuncX.apply(m_xHat.x, K.times(residualFuncY.apply(y, h.apply(m_xHat.x, u))));
        return new RandomVector<States>(xhat, m_P);
    }

    /**
     * Project the model into the future with a new control input u.
     *
     * @param m_xHat state
     * @param u         New control input from controller.
     * @param dtSeconds Timestep for prediction.
     */
    public RandomVector<States> predict(RandomVector<States> m_xHat, Matrix<Inputs, N1> u, double predictionDtSec) {
        Matrix<States, N1> xhat = NumericalIntegration.rk4(m_f, m_xHat.x, u, predictionDtSec);
        return new RandomVector<States>(xhat, m_P);
    }

    public Matrix<States, States> getP() {
        return m_P;
    }

    public double getP(int row, int col) {
        return m_P.get(row, col);
    }
}
