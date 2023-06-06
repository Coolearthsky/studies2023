package org.team100.lib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Fusing random variables is not a straightforward problem.
 * 
 * Here's a good survey of approaches: https://arxiv.org/pdf/2202.11633.pdf
 * 
 * In particular, the variables to be fused are certainly not independent, which
 * means the usual formulas are invalid, e.g. for mean a,b,c and covariance A,
 * B, C:
 * 
 * c = C*(a/A + b/B)
 * C = 1/(1/A + 1/B)
 * 
 * These are not true if A and B are correlated.
 * 
 * The usual strategy, suggested by the survey above and
 * 
 * https://apps.dtic.mil/sti/pdfs/ADA394392.pdf
 * 
 * and here
 * 
 * https://arxiv.org/pdf/1012.4795.pdf
 * 
 * is to invent a parameter, w, so that:
 * 
 * C = 1/(w/A + (1-w)/B)
 * c = C*(wa/A + (1-w)b/B)
 * 
 * the choice of w involves some optimization, which I don't really want to do,
 * so, let's just say it's a constant.
 */
public class RandomVectorFusor<States extends Num> {
    /** I think this is the most conservative possible guess. */
    private static final double w = 0.5;

    RandomVector<States> fuse(RandomVector<States> a, RandomVector<States> b) {
        Matrix<States, N1> ax = a.x;
        Matrix<States, N1> bx = b.x;
        Matrix<States, States> aP = a.P;
        Matrix<States, States> bP = b.P;
        Matrix<States, States> cP = aP.inv().times(w).plus(bP.inv().times(1 - w)).inv();
        Matrix<States, N1> cx = cP.times(aP.inv().times(ax).times(w).plus(bP.inv().times(bx).times(1 - w)));
        return new RandomVector<States>(cx, cP);
    }

}
