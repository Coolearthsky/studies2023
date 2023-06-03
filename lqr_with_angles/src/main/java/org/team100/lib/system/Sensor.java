package org.team100.lib.system;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * There can be any number of sensors viewing a system, so it's a separate
 * interface.
 */
public interface Sensor<States extends Num, Inputs extends Num, Rows extends Num> {
    /** A sensor can return any number of measurement Rows at once. */
    public Matrix<Rows, N1> h(Matrix<States, N1> x, Matrix<Inputs, N1> u);

    /**
     * Measurement residual, e.g. subtraction, used by the filter to compare the
     * measurement with the estimated measurement.
     */
    public Matrix<Rows, N1> yResidual(Matrix<Rows, N1> a, Matrix<Rows, N1> b);
}
