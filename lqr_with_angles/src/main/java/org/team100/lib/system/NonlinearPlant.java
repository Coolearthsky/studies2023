package org.team100.lib.system;

import org.team100.lib.math.RandomVector;

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
    public RandomVector<States> f(RandomVector<States> x, Matrix<Inputs, N1> u);

    /** Inverse with respect to u, for feedforward */
    public Matrix<Inputs, N1> finv(RandomVector<States> x, RandomVector<States> xdot);

    /** Measure all states; this is really only used for initialization. */
    public Sensor<States, Inputs, Outputs> full();

    /** Control limit */
    public Matrix<Inputs, N1> limit(Matrix<Inputs, N1> u);

    public Nat<States> states();

    public Nat<Inputs> inputs();

    public Nat<Outputs> outputs();
}
