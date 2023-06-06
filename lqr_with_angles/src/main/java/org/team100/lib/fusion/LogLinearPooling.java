package org.team100.lib.fusion;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * A geometric mixture of probability functions;
 * 
 * P = product(P_i^w_i)
 * 
 * For gaussian random variables, for mean a,b,c and covariance A, B, C:
 * 
 * c = C*(wa/A + (1-w)b/B)
 * C = 1/(w/A + (1-w)/B)
 * 
 * Notice that the aggregate mean is weighted by the covariances. This matches
 * the intuition that more-precise estimates should win. On the other hand, the
 * aggregate covariance doesn't depend on the means, which seems
 * counterintuitive.
 */
public class LogLinearPooling<States extends Num> implements Pooling<States> {
    // In covariance intersection, the choice of w minimizes C, but for now let's
    // just choose a number; it won't be optimal but I don't think it matters.
    private static final double w = 0.5;

    public RandomVector<States> fuse(RandomVector<States> a, RandomVector<States> b) {
        Matrix<States, N1> ax = a.x;
        Matrix<States, N1> bx = b.x;
        Matrix<States, States> aP = a.P;
        Matrix<States, States> bP = b.P;

        Matrix<States, States> cP = aP.inv().times(w).plus(bP.inv().times(1 - w)).inv();
        Matrix<States, N1> cx = cP.times(aP.inv().times(ax).times(w).plus(bP.inv().times(bx).times(1 - w)));

        return new RandomVector<States>(cx, cP);
    }
}
