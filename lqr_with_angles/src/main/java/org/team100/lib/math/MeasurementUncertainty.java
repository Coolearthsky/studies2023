package org.team100.lib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N2;

/**
 * Represents uncertainty, v in the measurement equation:
 * 
 * y = h(x, t) + v(t)
 * 
 * Represents the beliefs of of the sensors in a plant.
 * 
 * https://en.wikipedia.org/wiki/Measurement_uncertainty
 */
public class MeasurementUncertainty<States extends Num> {
    public final Matrix<States, States> P;

    public MeasurementUncertainty(Matrix<States, States> P) {
        this.P = P;
    }

    public static MeasurementUncertainty<N2> for2(double w1, double w2) {
        Matrix<N2,N2> w = new Matrix<>(Nat.N2(),Nat.N2());
        w.set(0,0,w1);
        w.set(1,1,w2);
        return new MeasurementUncertainty<>(w);
    }
}
