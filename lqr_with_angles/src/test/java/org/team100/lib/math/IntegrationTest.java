package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class IntegrationTest {
    private static final double kDelta = 0.001;

    public RandomVector<N1> f1(RandomVector<N1> x, Matrix<N1, N1> u) {
        Matrix<N1, N1> xx = new Matrix<>(Nat.N1(), Nat.N1());
        Matrix<N1, N1> p = new Matrix<>(Nat.N1(), Nat.N1());
        p.set(0, 0, 1);
        return new RandomVector<>(xx, p);
    }

    public RandomVector<N2> f2(RandomVector<N2> x, Matrix<N1, N1> u) {
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 1);
        p.set(1, 1, 1);
        return new RandomVector<>(xx, p);
    }

    @Test
    public void testRandomVectorIntegration1() {
        Matrix<N1, N1> x = new Matrix<>(Nat.N1(), Nat.N1());
        x.set(0, 0, 1);
        Matrix<N1, N1> p = new Matrix<>(Nat.N1(), Nat.N1());
        p.set(0, 0, 1);
        RandomVector<N1> v1 = new RandomVector<>(x, p);
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        // big time step here to see the effect
        RandomVector<N1> i1 = Integration.rk4(this::f1, v1, u, 1);
        // same as input
        assertArrayEquals(new double[] { 1 }, i1.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1.278 }, i1.P.getData(), kDelta);
    }

    @Test
    public void testRandomVectorIntegration2() {
        Matrix<N2, N1> x = new Matrix<>(Nat.N2(), Nat.N1());
        x.set(0, 0, 1);
        x.set(1, 0, 1);
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(), Nat.N2());
        p.set(0, 0, 1);
        p.set(0, 1, 0);
        p.set(1, 0, 0);
        p.set(1, 1, 1);
        RandomVector<N2> v2 = new RandomVector<>(x, p);
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1());
        // big time step here to see the effect
        RandomVector<N2> i2 = Integration.rk4(this::f2, v2, u, 1);
        assertArrayEquals(new double[] { 1, 1 }, i2.x.getData(), kDelta);
        assertArrayEquals(new double[] { 1.278, 0, 0, 1.278 }, i2.P.getData(), kDelta);
    }
}
