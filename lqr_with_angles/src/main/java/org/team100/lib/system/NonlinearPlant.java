package org.team100.lib.system;

import org.team100.lib.math.RandomVector;
import org.team100.lib.math.WhiteNoiseVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Represents a plant with nonlinear dynamics.
 * 
 * xdot = f(x,u) + w
 * y = h(x,u)
 * 
 * where
 * 
 * x: system state
 * u: control input
 * y: measurement output
 * w: noise process
 */
public interface NonlinearPlant<States extends Num, Inputs extends Num, Outputs extends Num> {
    /** State evolution */
    RandomVector<States> f(RandomVector<States> x, Matrix<Inputs, N1> u);

    /** Inverse of f with respect to u, for feedforward. */
    Matrix<Inputs, N1> finvWrtU(RandomVector<States> x, RandomVector<States> xdot);

    /**
     * Inverse of f with respect to x, for trending measurement. It's not that
     * important to perfectly invert f; if you get the velocity term, that's
     * probably enough. Use wide variances for unknowns.
     */
    RandomVector<States> finvWrtX(RandomVector<States> xdot, Matrix<Inputs, N1> u);

    /** Measurement from state. Use wide variances for unknowns. */
    RandomVector<Outputs> h(RandomVector<States> x, Matrix<Inputs, N1> u);

    /**
     * Inverse of h with respect to x. Usually both h and hinv are identity.
     */
    RandomVector<States> hinv(RandomVector<Outputs> y, Matrix<Inputs, N1> u);

    /**
     * "Process noise" aka disturbance.
     */
    WhiteNoiseVector<States> xi();

    /** Control limit */
    Matrix<Inputs, N1> limit(Matrix<Inputs, N1> u);

    Nat<States> states();

    Nat<Inputs> inputs();

    Nat<Outputs> outputs();
}
