package org.team100.lib.fusion;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * An arithmetic mixture of probability functions:
 * 
 * P = sum(w_i * P_i)
 * 
 * For gaussian random variables, for mean a,b,c and covariance A, B, C:
 * 
 * c = wa + (1-w)b
 * C = w(A + (a-c)^2) + (1-w)(B + (b-c)^2)
 * 
 * Notice that the aggregate mean doesn't depend on the covariances, which
 * is nonintuitive (shouldn't more-precise estimates have more weight?)
 * The aggregate covariance *does* depend on the means. This matches the
 * intuition that, if two estimates of the same quantity have different means,
 * the underlying quantity probably has a wide variance.
 */
public class LinearPooling<States extends Num> implements Pooling<States> {
    private static final double w = 0.5;

    public RandomVector<States> fuse(RandomVector<States> a, RandomVector<States> b) {
        Matrix<States, N1> ax = a.x;
        Matrix<States, N1> bx = b.x;
        Matrix<States, States> aP = a.P;
        Matrix<States, States> bP = b.P;

        Matrix<States, N1> cx = ax.times(w).plus(bx.times(1 - w));
        Matrix<States, N1> aE = ax.minus(cx);
        Matrix<States, N1> bE = bx.minus(cx);
        Matrix<States, States> cPA = aP.plus(aE.times(aE.transpose())).times(w);
        Matrix<States, States> cPB = bP.plus(bE.times(bE.transpose())).times(1 - w);
        Matrix<States, States> cP = cPA.plus(cPB);

        return new RandomVector<States>(cx, cP);
    }

}
