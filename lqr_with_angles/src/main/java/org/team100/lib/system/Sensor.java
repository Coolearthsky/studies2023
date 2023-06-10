package org.team100.lib.system;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Represents measurement:
 * 
 * y = h(x,u)
 * 
 * The measurement function, h, includes the notion of uncertainty.
 * 
 * There can be any number of sensors viewing a system, so it's a separate
 * interface.
 * 
 * TODO: combine this with the system interface, instead of variable measurement
 * rowcount, use high variance for don't-know values.
 * 
 * @param <States>  state dimensions of the observed system
 * @param <Inputs>  input dimensions of the observed system
 * @param <Outputs> output measurement dimensions of the sensor
 */
public interface Sensor<States extends Num, Inputs extends Num, Outputs extends Num> {
    // /** Dimensionality of this sensor */
    // public Nat<Rows> rows();

    /**
     * A sensor always returns all the rows but maybe with large don't-know
     * variances.
     */
    public RandomVector<Outputs> h(RandomVector<States> x, Matrix<Inputs, N1> u);

    /** Inverse of the above. Don't use noninvertible sensors. */
    public RandomVector<States> hinv(RandomVector<Outputs> y, Matrix<Inputs, N1> u);

    /**
     * Measurement residual, e.g. subtraction, used by the filter to compare the
     * measurement with the estimated measurement.
     */
    public RandomVector<Outputs> yResidual(RandomVector<Outputs> a, RandomVector<Outputs> b);

    // /** Standard deviations of the measurement, used by the filter. */
    // public Matrix<Rows, N1> stdev();
}
