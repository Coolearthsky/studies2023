package org.team100.lib.estimator;

import java.util.function.BiFunction;

import org.team100.lib.math.RandomVector;
import org.team100.lib.math.WhiteNoiseVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Solves the initial value problem with random vectors via integration.
 */
public class IntegratingPredictor<States extends Num, Inputs extends Num> {

    /** Integrate f forwards and add xi noise. */
    public RandomVector<States> predictWithNoise(
            BiFunction<RandomVector<States>, Matrix<Inputs, N1>, RandomVector<States>> f,
            RandomVector<States> x,
            Matrix<Inputs, N1> u,
            WhiteNoiseVector<States> xi,
            double dtS) {
        return addNoise(predict(f, x, u, dtS), xi, dtS);
    }

    /** Runge-Kutta 4 for random vectors. */
    public RandomVector<States> predict(
            BiFunction<RandomVector<States>, Matrix<Inputs, N1>, RandomVector<States>> f,
            RandomVector<States> x,
            Matrix<Inputs, N1> u,
            double dtS) {
        final var h = dtS;

        RandomVector<States> k1 = f.apply(x, u);
        RandomVector<States> k2 = f.apply(x.plus(k1.times(h * 0.5)), u);
        RandomVector<States> k3 = f.apply(x.plus(k2.times(h * 0.5)), u);
        RandomVector<States> k4 = f.apply(x.plus(k3.times(h)), u);

        return x.plus((k1.plus(k2.times(2.0)).plus(k3.times(2.0)).plus(k4)).times(h / 6.0));
    }

    /** Noise integration produces variance of t */
    public RandomVector<States> addNoise(RandomVector<States> x, WhiteNoiseVector<States> xi, double dtSeconds) {
        Matrix<States, States> P = xi.P.copy();
        P = P.times(dtSeconds);
        return x.make(x.x, x.P.plus(P));
    }

}
