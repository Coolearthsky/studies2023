package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;

public class RandomVectorTest {
    private static final double kDelta = 0.001;
    @Test
    public void testVacuous() {
        Matrix<N1, N1> xV = VecBuilder.fill(1);
        Matrix<N1, N1> PV = VecBuilder.fill(2);
        RandomVector<N1> v = new RandomVector<>(xV, PV);
        assertEquals(1, v.x.get(0,0), kDelta);
        assertEquals(2, v.P.get(0,0), kDelta);
    }
}
