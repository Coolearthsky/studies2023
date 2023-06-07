package org.team100.lib.math;

import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** for random variables */
public class Integration {
    public static <States extends Num, Inputs extends Num> RandomVector<States> rk4(
            BiFunction<RandomVector<States>, Matrix<Inputs, N1>, RandomVector<States>> f,
            RandomVector<States> x,
            Matrix<Inputs, N1> u,
            double dtSeconds) {
        final var h = dtSeconds;

        RandomVector<States> k1 = f.apply(x, u);
        RandomVector<States> k2 = f.apply(x.plus(k1.times(h * 0.5)), u);
        RandomVector<States> k3 = f.apply(x.plus(k2.times(h * 0.5)), u);
        RandomVector<States> k4 = f.apply(x.plus(k3.times(h)), u);

        return x.plus((k1.plus(k2.times(2.0)).plus(k3.times(2.0)).plus(k4)).times(h / 6.0));
    }
}
