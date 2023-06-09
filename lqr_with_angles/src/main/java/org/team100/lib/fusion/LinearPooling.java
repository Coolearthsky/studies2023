package org.team100.lib.fusion;

import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * An arithmetic mixture of probability functions:
 * 
 * P = sum(w_i * P_i)
 * 
 * For gaussian random variables, for mean a,b,c and covariance A, B, C, the
 * mean of the mixture is just the weighted sum:
 * 
 * c = wa + (1-w)b
 * 
 * The variance is also the weighted sum, with an extra term representing the
 * dispersion of the means [1]:
 * 
 * C = w(A + (a-c)^2) + (1-w)(B + (b-c)^2)
 * 
 * more transparently written [2]:
 * 
 * C = wA + (1-w)B + w(1-w)(a-b)^2
 * 
 * Note that a mixture of gaussians is not at all gaussian! Consider
 * widely-spaced low-variance inputs will yield a bimodal mixture. So this is
 * kind of a hack.
 * 
 * How does this pooling behave?
 * 
 * The aggregate mean doesn't depend on the covariances, which seems wrong:
 * shouldn't more-precise estimates have more influence?
 * 
 * The aggregate covariance depends on the means, which seems right: if two
 * estimates of the same quantity have different means, that quantity probably
 * has a wide variance.
 * 
 * This pooling method works fine with zero variances.
 * 
 * [1] https://arxiv.org/pdf/2202.11633.pdf
 * [2]
 * https://stats.stackexchange.com/questions/16608/what-is-the-variance-of-the-weighted-mixture-of-two-gaussians
 */
public abstract class LinearPooling<States extends Num> implements Pooling<States> {

    RandomVector<States> fuse(
            RandomVector<States> a,
            Matrix<States, States> pa,
            RandomVector<States> b,
            Matrix<States, States> pb) {
        // TODO: make these checks "test mode only" to avoid throwing in matches

        Matrix<States, States> sumOfWeight = pa.plus(pb);
        if (!MatrixFeatures_DDRM.isIdentity(sumOfWeight.getStorage().getDDRM(), 0.001)) {
            throw new IllegalArgumentException("weights do not sum to one.\npa:\n "
                    + pa.toString() + "\npb:\n " + pb.toString());
        }

        Matrix<States, N1> ax = a.x;
        Matrix<States, N1> bx = b.x;
        Matrix<States, States> aP = a.P;
        Matrix<States, States> bP = b.P;

        // TODO: make these checks "test mode only" to avoid throwing in matches
        if (!MatrixFeatures_DDRM.isSymmetric(aP.getStorage().getDDRM())) {
            throw new IllegalArgumentException("aP is not symmetric.\n" + aP.toString());
        }
        if (!MatrixFeatures_DDRM.isPositiveSemidefinite(aP.getStorage().getDDRM())) {
            throw new IllegalArgumentException("aP is not positive semidefinite.\n" + aP.toString());
        }
        if (!MatrixFeatures_DDRM.isSymmetric(bP.getStorage().getDDRM())) {
            throw new IllegalArgumentException("bP is not symmetric.\n" + bP.toString());
        }
        if (!MatrixFeatures_DDRM.isPositiveSemidefinite(bP.getStorage().getDDRM())) {
            throw new IllegalArgumentException("bP is not positive semidefinite.\n" + bP.toString());
        }

        // weighted mean:
        Matrix<States, N1> cx = pa.times(ax).plus(pb.times(bx));

        // variance, transparent version
        Matrix<States, States> cPA = pa.times(aP);
        Matrix<States, States> cPB = pb.times(bP);
        // dispersion term
        Matrix<States, N1> d = ax.minus(bx);
        double d2 = d.transpose().times(d).get(0,0);
        Matrix<States, States> papb = pa.times(pb);
        Matrix<States, States> dP = papb.times(d2);
        Matrix<States, States> cP = cPA.plus(cPB).plus(dP);

        return new RandomVector<States>(cx, cP);
    }

    /**
     * This is really just for testing; supply scalar weights, this makes diagonal
     * matrices.
     */
    RandomVector<States> fuse(RandomVector<States> a, double pa, RandomVector<States> b, double pb) {
        Matrix<States, States> pamat = a.P.copy();
        Matrix<States, States> pbmat = b.P.copy();
        pamat.fill(0);
        pbmat.fill(0);
        for (int i = 0; i < pamat.getNumCols(); ++i) {
            pamat.set(i, i, pa);
            pbmat.set(i, i, pb);
        }
        return fuse(a, pamat, b, pbmat);
    }

    /**
     * Weights should add to one.
     * 
     * @param pa a weight
     * @param pb b weight
     */
    RandomVector<States> fuseold(RandomVector<States> a, double pa, RandomVector<States> b, double pb) {
        Matrix<States, N1> ax = a.x;
        Matrix<States, N1> bx = b.x;
        Matrix<States, States> aP = a.P;
        Matrix<States, States> bP = b.P;

        // weighted mean:
        Matrix<States, N1> cx = ax.times(pa).plus(bx.times(pb));

        // variance, opaque version:
        // Matrix<States, N1> aE = ax.minus(cx);
        // Matrix<States, N1> bE = bx.minus(cx);
        // Matrix<States, States> cPA = aP.plus(aE.times(aE.transpose())).times(pa);
        // Matrix<States, States> cPB = bP.plus(bE.times(bE.transpose())).times(pb);
        // Matrix<States, States> cP = cPA.plus(cPB);

        // variance, transparent version:
        Matrix<States, States> cPA = aP.times(pa);
        Matrix<States, States> cPB = bP.times(pb);
        // dispersion term
        Matrix<States, N1> d = ax.minus(bx);
        Matrix<States, States> d2 = d.times(d.transpose());
        Matrix<States, States> dP = d2.times(pa).times(pb);
        Matrix<States, States> cP = cPA.plus(cPB).plus(dP);

        return new RandomVector<States>(cx, cP);
    }

}
