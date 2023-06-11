package org.team100.lib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * A vector-valued gaussian random variable, which represents a belief about a
 * hidden state.
 * 
 * TODO: make distinct classes for state vs measurement
 */
public class RandomVector<States extends Num> {
    public final Matrix<States, N1> x;
    public final Matrix<States, States> P;

    public RandomVector(Matrix<States, N1> x, Matrix<States, States> P) {
        this.x = x;
        this.P = P;
    }

    /** instantiation that preserves type */
    public RandomVector<States> make(Matrix<States, N1> x, Matrix<States, States> P) {
        return new RandomVector<>(x, P);
    }

    public RandomVector<States> copy() {
        return make(x.copy(), P.copy());
    }

    /**
     * Mean and covariance are simply added, which corresponds to assuming the
     * variables are independent.
     */
    public RandomVector<States> plus(RandomVector<States> b) {
        return make(x.plus(b.x), P.plus(b.P));
    }

    /**
     * Mean is subtracted, covariance is *added* which corresponds to assuming the
     * variables are independent.
     */
    public RandomVector<States> minus(RandomVector<States> b) {
        return make(x.minus(b.x), P.plus(b.P));
    }

    /** euclidean version */
    public Matrix<States, N1> xplus(Matrix<States, N1> otherx) {
        return this.x.plus(otherx);
    }

    /**
     * euclidean version, makes the expressions a little cleaner to have both plus
     * and minus
     */
    public Matrix<States, N1> xminus(Matrix<States, N1> otherx) {
        return this.x.minus(otherx);
    }

    /**
     * Weighted average with another variable.
     * 
     * @param weight the weight of the other variable, assuming the weight of this
     *               variable is 1-weight.
     * @return this + weight * (other - this)
     */
    public RandomVector<States> combine(Matrix<States, States> weight, RandomVector<States> other) {
        Matrix<States, N1> xx = xplus(weight.times(other.xminus(this.x)));
        Matrix<States, States> PP = this.P.plus(weight.times(other.P.minus(this.P)));
        return make(xx, PP);
    }

    /**
     * Scalar multiplication. Remember that the scalar is *squared* before applying
     * to the covariance.
     */
    public RandomVector<States> times(double d) {
        return make(x.times(d), P.times(d * d));
    }

    @Override
    public String toString() {
        return "RandomVector [x=" + x + ", P=" + P + "]";
    }

}
