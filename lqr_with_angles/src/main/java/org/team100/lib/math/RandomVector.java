package org.team100.lib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * A vector-valued gaussian random variable, which represents a belief about a hidden state.
 */
public class RandomVector<States extends Num> {
    public final Matrix<States, N1> x;
    public final Matrix<States, States> P;

    public RandomVector(Matrix<States, N1> x, Matrix<States, States> P) {
        this.x = x;
        this.P = P;
    }

    public final RandomVector<States> copy() {
        return new RandomVector<>(x.copy(), P.copy());
    }

    /**
     * Mean is subtracted, covariance is *added* which corresponds to assuming the
     * variables are independent.
     */
    public RandomVector<States> minus(RandomVector<States> b) {
        return new RandomVector<>(x.minus(b.x), P.plus(b.P));
    }

    /**
     * Mean and covariance are simply added, which corresponds to assuming the
     * variables are independent.
     */
    public RandomVector<States> plus(RandomVector<States> b) {
        return new RandomVector<>(x.plus(b.x), P.plus(b.P));
    }

    /**
     * Scalar multiplication. Remember that the scalar is *squared* before applying
     * to the covariance.
     */
    public RandomVector<States> times(double d) {
        return new RandomVector<>(x.times(d), P.times(d * d));
    }

    @Override
    public String toString() {
        return "RandomVector [x=" + x + ", P=" + P + "]";
    }

    
}
