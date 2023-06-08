package org.team100.lib.fusion;

import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * similar to the democratic case
 */
public class VarianceWeightedLinearPoolingTest extends PoolingTest {
    private static final LinearPooling<N1> p1 = new VarianceWeightedLinearPooling<N1>();
    private static final LinearPooling<N2> p2 = new VarianceWeightedLinearPooling<N2>();

    @Test
    public void testUnanimity1() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 1);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        // fuse with yourself => no change
        assert1(cV, 0, 1);
    }

    @Test
    public void testUnanimity2() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> bV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        // fuse with yourself => no change
        assert2(cV, 0, 0, 1, 0, 0, 1);
    }

    @Test
    public void testDifferentMeans() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 1);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        // aggregate mean is right in the middle
        // aggregate variance is a bit bigger since the means are different
        assert1(cV, 0.5, 1.25);
    }

    @Test
    public void testDifferentVariance() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 2);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        // this is like the log-linear case now since it weights by inverse variance
        assert1(cV, 0.0, 1.333);
    }

    @Test
    public void testDifferent1() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 2);
        RandomVector<N1> cV = p1.fuse(aV, bV);
        // the mean is like log-linear, weighted by inverse variance
        // the variance is bigger than log-linear due to the dispersion term
        assert1(cV, 0.333, 1.555);
    }

    @Test
    public void testDifferent2NoCorrelation() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> bV = v2(1, 1, 2, 0, 0, 2);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        // the mean is like log-linear, weighted by inverse variance
        // the variance is bigger than log-linear due to the dispersion term
        assert2(cV, 0.333, 0.333, 1.555, 0.222, 0.222, 1.555);
    }

    @Test
    public void testDifferent2bNoCorrelation() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 2);
        RandomVector<N2> bV = v2(1, 1, 2, 0, 0, 1);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        // mean leans towards the tighter variance
        assert2(cV, 0.333, 0.666, 1.555, 0.222, 0.222, 1.555);
    }

    @Test
    public void testDifferent2WithCorrelation() {
        // different means, different variances
        RandomVector<N2> aV = v2(0, 0, 1, 0.5, 0.5, 1);
        RandomVector<N2> bV = v2(1, 1, 2, 0.5, 0.5, 2);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        assert2(cV, 0.375, 0.375, 1.547, 0.797, 0.797, 1.547);
    }

    @Test
    public void testDifferent2bWithCorrelation() {
        // some off-diagonal covariance terms and different variances
        RandomVector<N2> aV = v2(0, 0, 1, 0.5, 0.5, 2);
        RandomVector<N2> bV = v2(1, 1, 2, 0.5, 0.5, 1);
        RandomVector<N2> cV = p2.fuse(aV, bV);
        // mean leans towards the tighter variance
        // off-diagonals make this effect stronger
        assert2(cV, 0.25, 0.75, 1.531, 0.781, 0.781, 1.531);
    }

    @Test
    public void testAsymmetricCovariance() {
        // some off-diagonal covariance terms :-)
        // this is not symmetric
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0.5, 1);
        RandomVector<N2> bV = v2(1, 1, 2, 1, 1, 2);
        assertThrows(IllegalArgumentException.class, () -> p2.fuse(aV, bV));
    }

    @Test
    public void testZeroVariance() {
        // can't invert this one
        RandomVector<N1> aV = v1(0, 0);
        RandomVector<N1> bV = v1(1, 1);
        // this is like the log-linear case now since it inverts the variances
        assertThrows(IllegalArgumentException.class, () -> p1.fuse(aV, bV));
    }

    @Test
    public void testOKWeights() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> bV = v2(1, 1, 1, 0, 0, 1);
        Matrix<N2, N2> pa = m2(0.5, 0, 0, 0.5);
        Matrix<N2, N2> pb = m2(0.5, 0, 0, 0.5);
        RandomVector<N2> cV = p2.fuse(aV, pa, bV, pb);
        // equal weight => mean is arithmetic, variance grows because of mean
        // dispersion, also note off-diagonals
        assert2(cV, 0.5, 0.5, 1.25, 0.25, 0.25, 1.25);
    }

    @Test
    public void testBadWeights() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1);
        RandomVector<N2> bV = v2(1, 1, 1, 0, 0, 1);
        // these do not add to identity
        Matrix<N2, N2> pa = m2(0.5, 0, 0, 0.5);
        Matrix<N2, N2> pb = m2(1, 0, 0, 1);
        assertThrows(IllegalArgumentException.class, () -> p2.fuse(aV, pa, bV, pb));
    }
}
