package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class JacobianTest {
    private static final double kDelta = 0.001;
    public RandomVector<N2> f(RandomVector<N2> x, Matrix<N1,N1> u) {
        return x;
    }

    @Test
    public void testJacobian() {
        RandomVector<N2> x = new RandomVector<>(new Matrix<>(Nat.N2(), Nat.N1()), new Matrix<>(Nat.N2(), Nat.N2()));
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        Matrix<N2, N2> A = Jacobian.numericalJacobianX(Nat.N2(), Nat.N2(), this::f, x, u);
        assertArrayEquals(new double[] {1, 0, 0, 1}, A.getData(), kDelta);
    }
}
