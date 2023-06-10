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
 * If you're doing non-Euclidean geometry, you'd better handle that in your
 * variable class.
 * 
 * [1] https://arxiv.org/pdf/2202.11633.pdf
 * [2]
 * https://stats.stackexchange.com/questions/16608/what-is-the-variance-of-the-weighted-mixture-of-two-gaussians
 */
public abstract class LinearPooling<States extends Num> implements Pooling<States> {
    /**
     * Note that a and b could involve non-cartesian dimensions, e.g. angle
     * wrapping.
     * The caller needs to handle normalization.
     */
    RandomVector<States> fuse(
            RandomVector<States> a,
            Matrix<States, States> pa,
            RandomVector<States> b,
            Matrix<States, States> pb) {
        // TODO: in "comp mode" make these checks substitute some acceptable behavior
        // like some simpler average

        Matrix<States, States> sumOfWeight = pa.plus(pb);
        if (!MatrixFeatures_DDRM.isIdentity(sumOfWeight.getStorage().getDDRM(), 0.001)) {
            throw new IllegalArgumentException("weights do not sum to one.\npa:\n "
                    + pa.toString() + "\npb:\n " + pb.toString());
        }
        if (!MatrixFeatures_DDRM.isSymmetric(a.P.getStorage().getDDRM())) {
            throw new IllegalArgumentException("aP is not symmetric.\n" + a.P.toString());
        }
        if (!MatrixFeatures_DDRM.isPositiveSemidefinite(a.P.getStorage().getDDRM())) {
            throw new IllegalArgumentException("aP is not positive semidefinite.\n" + a.P.toString());
        }
        if (!MatrixFeatures_DDRM.isSymmetric(b.P.getStorage().getDDRM())) {
            throw new IllegalArgumentException("bP is not symmetric.\n" + b.P.toString());
        }
        if (!MatrixFeatures_DDRM.isPositiveSemidefinite(b.P.getStorage().getDDRM())) {
            throw new IllegalArgumentException("bP is not positive semidefinite.\n" + b.P.toString());
        }

        Matrix<States, States> dispersionTerm = dispersionCovariance(a, pa, b, pb);
        RandomVector<States> cc = a.combine(pb, b);
        return a.make(cc.x, cc.P.plus(dispersionTerm));
    }

    /** Covariance of the mixture due to dispersion in the means. */
    private Matrix<States, States> dispersionCovariance(
            RandomVector<States> a,
            Matrix<States, States> pa,
            RandomVector<States> b,
            Matrix<States, States> pb) {
        Matrix<States, N1> d = a.xminus(b.x);
        Matrix<States, States> ddiag = pa.copy();
        ddiag.fill(0);
        for (int i = 0; i < ddiag.getNumCols(); ++i) {
            ddiag.set(i, i, d.get(i, 0));
        }
        Matrix<States, States> d2 = ddiag.times(ddiag);
        Matrix<States, States> papb = pa.times(pb);
        Matrix<States, States> dispersionTerm = papb.times(d2);
        return dispersionTerm;
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
}
