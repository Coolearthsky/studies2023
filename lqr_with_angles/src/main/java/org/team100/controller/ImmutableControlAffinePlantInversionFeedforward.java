package org.team100.controller;

import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalJacobian;

/**
 * Constructs a control-affine plant inversion model-based feedforward from
 * given model dynamics.
 *
 * <p>
 * If given the vector valued function as f(x, u) where x is the state vector
 * and u is the input
 * vector, the B matrix(continuous input matrix) is calculated through a {@link
 * edu.wpi.first.math.system.NumericalJacobian}. In this case f has to be
 * control-affine (of the
 * form f(x) + Bu).
 *
 * <p>
 * The feedforward is calculated as <strong> u_ff = B<sup>+</sup> (rDot -
 * f(x))</strong>, where
 * <strong> B<sup>+</sup> </strong> is the pseudoinverse of B.
 *
 * <p>
 * This feedforward does not account for a dynamic B matrix, B is either
 * determined or supplied
 * when the feedforward is created and remains constant.
 *
 * <p>
 * For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 * 
 * This version is just like the WPI version but without all the
 * result-retaining part, because immutability is good.
 */
public class ImmutableControlAffinePlantInversionFeedforward<States extends Num, Inputs extends Num> {
    /** The current reference state. */
    private Matrix<States, N1> m_r;

    private final Matrix<States, Inputs> m_B;
    private final Nat<Inputs> m_inputs;
   private final Matrix<Inputs, N1> uZero;
    private final double m_dt;

    /** The model dynamics. */
    private final BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> m_f;

    /**
     * Constructs a feedforward with given model dynamics as a function of state and
     * input.
     *
     * @param states    A {@link Nat} representing the number of states.
     * @param inputs    A {@link Nat} representing the number of inputs.
     * @param f         A vector-valued function of x, the state, and u, the input,
     *                  that returns the derivative of the state vector. HAS to be
     *                  control-affine (of the form f(x) + Bu).
     * @param dtSeconds The timestep between calls of calculate().
     */
    public ImmutableControlAffinePlantInversionFeedforward(
            Nat<States> states,
            Nat<Inputs> inputs,
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            double dtSeconds) {
        m_dt = dtSeconds;
        m_f = f;
        m_inputs = inputs;
        uZero = new Matrix<>(m_inputs, Nat.N1());

        Matrix<States, N1> x = new Matrix<>(states, Nat.N1());
        Matrix<Inputs, N1> u = new Matrix<>(inputs, Nat.N1());
        m_B = NumericalJacobian.numericalJacobianU(states, inputs, m_f, x, u);

        m_r = new Matrix<>(states, Nat.N1());

        reset();
    }

    /**
     * Resets the feedforward with a specified initial state vector.
     *
     * @param initialState The initial state vector.
     */
    public void reset(Matrix<States, N1> initialState) {
        m_r = initialState;
    }

    /** Resets the feedforward with a zero initial state vector. */
    public void reset() {
        m_r.fill(0.0);
    }

    /**
     * Calculate the feedforward with only the desired future reference. This uses
     * the internally
     * stored "current" reference.
     *
     * <p>
     * If this method is used the initial state of the system is the one set using
     * {@link
     * LinearPlantInversionFeedforward#reset(Matrix)}. If the initial state is not
     * set it defaults to
     * a zero vector.
     *
     * @param nextR The reference state of the future timestep (k + dt).
     * @return The calculated feedforward.
     */
    public Matrix<Inputs, N1> calculate(Matrix<States, N1> nextR) {
        return calculate(m_r, nextR);
    }

    /**
     * Calculate the feedforward with current and future reference vectors.
     *
     * @param r     The reference state of the current timestep (k).
     * @param nextR The reference state of the future timestep (k + dt).
     * @return The calculated feedforward.
     */
    public Matrix<Inputs, N1> calculate(Matrix<States, N1> r, Matrix<States, N1> nextR) {
        var rDot = (nextR.minus(r)).div(m_dt);

        Matrix<Inputs, N1>  uff = m_B.solve(rDot.minus(m_f.apply(r, uZero)));

        m_r = nextR;
        return uff;
    }
}
