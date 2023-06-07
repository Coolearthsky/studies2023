package org.team100.lib.system;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * There can be any number of sensors viewing a system, so it's a separate
 * interface.
 * 
 * @param <States> state dimensions of the observed system
 * @param <Inputs> input dimensions of the observed system
 * @param <Rows>   output measurement dimensions of the sensor
 */
public interface Sensor<States extends Num, Inputs extends Num, Rows extends Num> {
    /** Dimensionality of this sensor */
    public Nat<Rows> rows();

    /** A sensor can return any number of measurement Rows at once. */
    public RandomVector<Rows> h(RandomVector<States> x, Matrix<Inputs, N1> u);

    /**
     * Measurement residual, e.g. subtraction, used by the filter to compare the
     * measurement with the estimated measurement.
     */
    public RandomVector<Rows> yResidual(RandomVector<Rows> a, RandomVector<Rows> b);

    /** Standard deviations of the measurement, used by the filter. */
    public Matrix<Rows, N1> stdev();
}
