package org.team100.lib.estimator;

import java.util.function.BiFunction;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** State from measurement trend. */
public class TrendEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {

    public RandomVector<States> stateForMeasurementPair(
            Matrix<Inputs, N1> u,
            RandomVector<Outputs> y0,
            RandomVector<Outputs> y1,
            BiFunction<RandomVector<States>, Matrix<Inputs,N1>, RandomVector<States>> finv,
            BiFunction<RandomVector<Outputs>, Matrix<Inputs, N1>, RandomVector<States>> hinv,
            double dtS) {
        RandomVector<States> x0 = hinv.apply(y0, u);
        RandomVector<States> x1 = hinv.apply(y1, u);
        RandomVector<States> xdot = x1.minus(x0).times(1.0/dtS);
        RandomVector<States> x = finv.apply(xdot, u);
        return x;
    }

}
