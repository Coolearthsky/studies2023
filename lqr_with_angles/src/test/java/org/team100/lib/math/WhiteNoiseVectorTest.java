package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class WhiteNoiseVectorTest {
    private static final double kDelta = 0.001;
    @Test
    public void testIntegration1() {
        Matrix<N1, N1> p = new Matrix<>(Nat.N1(),Nat.N1());
        p.set(0,0,2);
        WhiteNoiseVector<N1> xi = new WhiteNoiseVector<>(p);
        RandomVector<N1> x = Integration.wiener(xi, 0.02);
        assertArrayEquals(new double[] {0}, x.x.getData(), kDelta);
        assertArrayEquals(new double[] {0.04}, x.P.getData(), kDelta);
    }
    @Test
    public void testIntegration2() {
        Matrix<N2, N2> p = new Matrix<>(Nat.N2(),Nat.N2());
        p.set(0,0,2);
        p.set(0,1,0);
        p.set(1,0,0);
        p.set(1,1,0.5);

        WhiteNoiseVector<N2> xi = new WhiteNoiseVector<>(p);
        RandomVector<N2> x = Integration.wiener(xi, 0.02);
        assertArrayEquals(new double[] {0, 0}, x.x.getData(), kDelta);
        assertArrayEquals(new double[] {0.04, 0, 0, 0.01}, x.P.getData(), kDelta);
    }
    
}
