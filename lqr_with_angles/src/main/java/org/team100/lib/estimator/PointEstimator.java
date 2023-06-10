package org.team100.lib.estimator;

import java.util.function.BiFunction;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** State from point measurements. */
public class PointEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {

    /**
     * Use the inverse h function to get the state corresponding to the measurement.
     */
    public RandomVector<States> stateForMeasurement(
            Matrix<Inputs, N1> u,
            RandomVector<Outputs> y,
            BiFunction<RandomVector<Outputs>, Matrix<Inputs, N1>, RandomVector<States>> hinv) {
        RandomVector<States> x = hinv.apply(y, u);
        return x;
    }

}
