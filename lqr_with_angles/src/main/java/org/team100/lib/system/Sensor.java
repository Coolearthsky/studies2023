package org.team100.lib.system;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
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
 * @param <States>  state dimensions of the observed system
 * @param <Inputs>  input dimensions of the observed system
 * @param <Outputs> output measurement dimensions of the sensor
 */
public interface Sensor<States extends Num, Inputs extends Num, Outputs extends Num> {
    /**
     * A sensor always returns all the rows but maybe with large don't-know
     * variances.
     */
    public RandomVector<Outputs> h(RandomVector<States> x, Matrix<Inputs, N1> u);

    /** Inverse of the above with respect to x. Don't use noninvertible sensors. */
    public RandomVector<States> hinv(RandomVector<Outputs> y, Matrix<Inputs, N1> u);
}
