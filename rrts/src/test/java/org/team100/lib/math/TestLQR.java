package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.DARE;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;

/**
 * Look at S matrices from linearized LQR.
 */
public class TestLQR {

    @Test
    public void testit() {
        Vector<N2> qelms = VecBuilder.fill(1.0, 1.0);
        Vector<N1> relms = VecBuilder.fill(1.0);

        Matrix<N2, N2> Q = StateSpaceUtil.makeCostMatrix(qelms);
        Matrix<N1, N1> R = StateSpaceUtil.makeCostMatrix(relms);

        Matrix<N2, N1> B = VecBuilder.fill(0, 1);

        for (double x1 = -1 * Math.PI; x1 < Math.PI; x1 += Math.PI / 4) {

            Matrix<N2, N2> A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, -1 * Math.cos(x1), 0);

            Pair<Matrix<N2, N2>, Matrix<N2, N1>> discABPair = Discretization.discretizeAB(A, B, 1);
            Matrix<N2, N2> discA = discABPair.getFirst();
            Matrix<N2, N1> discB = discABPair.getSecond();

            Matrix<N2, N2> S = DARE.dare(discA, discB, Q, R);

            Matrix<N1, N2> K = discB
                    .transpose()
                    .times(S)
                    .times(discB)
                    .plus(R)
                    .solve(discB.transpose().times(S).times(discA));
            // TODO: a better test, or delete it
            assertArrayEquals(new double[] { 0, 0 }, K.getData(), 10);
        }
    }

    @Test
    public void mult() {
        // because i forgot how to multiply
        Matrix<N2, N2> i = Matrix.mat(Nat.N2(), Nat.N2()).fill(1, 0, 0, 1); // identity
        assertArrayEquals(new double[] { 1, 0, 0, 1 }, i.getData());
        Matrix<N2, N1> x = VecBuilder.fill(5, 6);
        assertArrayEquals(new double[] { 5, 6 }, x.getData());
        Matrix<N2, N1> ix = i.times(x);
        assertArrayEquals(new double[] { 5, 6 }, ix.getData());
        Matrix<N1, N2> xi = x.transpose().times(i);
        assertArrayEquals(new double[] { 5, 6 }, xi.getData());
        Matrix<N1, N1> d = x.transpose().times(i).times(x);
        assertArrayEquals(new double[] { 61 }, d.getData());
        double dd = d.get(0, 0);
        assertEquals(61, dd);

    }

}
