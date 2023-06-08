package org.team100.lib.fusion;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.numbers.N1;

public class LinearPoolingTest extends PoolingTest {
    private static final Pooling<N1> p = new DemocraticLinearPooling<N1>();

    @Test
    public void testUnanimity() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // fuse with yourself => no change
        assert1(cV, 0, 1);
    }

    @Test
    public void testDifferentMeans() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate mean is right in the middle
        // aggregate variance is a bit bigger since the means are different
        assert1(cV, 0.5, 1.25);
    }

    @Test
    public void testDifferentVariance() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 2);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate mean is the same
        // variance is right in the middle of the estimate variances
        assert1(cV, 0.0, 1.5);
    }

    @Test
    public void testDifferent() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 2);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate mean is right in the middle
        // aggregate variance is a bit bigger, both mean and variance affect it
        assert1(cV, 0.5, 1.75);
    }

    @Test
    public void testZeroVariance() {
        RandomVector<N1> aV = v1(0, 0);
        RandomVector<N1> bV = v1(1, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate mean in the middle
        // aggregate variance is ... ok i guess?
        assert1(cV, 0.5, 0.75);
    }
}
