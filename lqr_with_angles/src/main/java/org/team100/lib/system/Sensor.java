package org.team100.lib.system;

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
    public Matrix<Rows, N1> h(Matrix<States, N1> x, Matrix<Inputs, N1> u);

    /**
     * Measurement residual, e.g. subtraction, used by the filter to compare the
     * measurement with the estimated measurement.
     */
    public Matrix<Rows, N1> yResidual(Matrix<Rows, N1> a, Matrix<Rows, N1> b);

    /** Standard deviations of the measurement, used by the filter. */
    public Matrix<Rows, N1> stdev();
}
