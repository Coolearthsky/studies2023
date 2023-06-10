package org.team100.lib.estimator;

import java.util.function.BiFunction;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** State from point measurements. */
public class PointEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final Matrix<Inputs, N1> m_uZero;

    public PointEstimator(Nat<Inputs> inputs) {
        // TODO: find another way to get this zero-input in here
        m_uZero = new Matrix<>(inputs, Nat.N1());
    }

    /**
     * Use the inverse h function to get the state corresponding to the measurement.
     */
    private RandomVector<States> stateForMeasurement(
            Matrix<Inputs, N1> u,
            RandomVector<Outputs> y,
            BiFunction<RandomVector<Outputs>, Matrix<Inputs, N1>, RandomVector<States>> hinv) {
        return hinv.apply(y, u);
    }

    /** in practice h is u-invariant */
    public RandomVector<States> stateForMeasurementWithZeroU(
            RandomVector<Outputs> y,
            BiFunction<RandomVector<Outputs>, Matrix<Inputs, N1>, RandomVector<States>> hinv) {
        return stateForMeasurement(m_uZero, y, hinv);
    }

}
