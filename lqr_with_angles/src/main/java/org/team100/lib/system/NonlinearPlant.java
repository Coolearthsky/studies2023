package org.team100.lib.system;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** Represents a plant with nonlinear dynamics. */
public interface NonlinearPlant<States extends Num, Inputs extends Num, Outputs extends Num> {
    /** State evolution */
    public Matrix<States, N1> f(Matrix<States, N1> x, Matrix<Inputs, N1> u);

    /**
     * State addition, used by the filter to correct the state towards the
     * measurement.
     */
    public Matrix<States, N1> xAdd(Matrix<States, N1> a, Matrix<States, N1> b);

    /**
     * Normalize state, used by predict, e.g. for angle wrapping.
     */
    public Matrix<States, N1> xNormalize(Matrix<States, N1> x);

    /**
     * State residual, e.g. subtraction, used by the controller to compare the
     * reference with the estimate.
     */
    public Matrix<States, N1> xResidual(Matrix<States, N1> a, Matrix<States, N1> b);

    /** Measure all states; this is really only used for initialization. */
    public Sensor<States, Inputs, Outputs> full();

    /**
     * Standard deviations of the state, used by the filter; i guess this represents
     * the stdev of the disturbance?
     */
    public Matrix<States, N1> stdev();

    /** Control limit */
    public Matrix<Inputs, N1> limit(Matrix<Inputs, N1> u);

    public Nat<States> states();

    public Nat<Inputs> inputs();

    public Nat<Outputs> outputs();
}
