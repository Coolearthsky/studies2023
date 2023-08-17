package org.team100.lib.prrts;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.unc.robotics.prrts.example.swingup.PendulumArena;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N2;

public class PendulumTest {
    @Test
    void testDist() {
        PendulumArena arena = new PendulumArena(new double[] { Math.PI, 0 }, 10);

        Matrix<N2, N2> S;
        S = arena.getS(new double[] { 0, 0 });
        // is it cos (d/dx)? or sin because it's just a coefficient?
        // i think it's sin.
        // cos
        // assertArrayEquals(new double[] { 6.9, -4.9, -4.9, 20.1 }, S.getData(), 0.1);
        // sin
        // assertArrayEquals(new double[] { 2.3, 1.1, 1.1, 2.6 }, S.getData(), 0.1);
        // no gravity
        assertArrayEquals(new double[] { 2.4, 1.1, 1.1, 2.6 }, S.getData(), 0.1);

        S = arena.getS(new double[] { 0.1, 0.1 });
        // cos
        // assertArrayEquals(new double[] { 6.4, -4.9, -4.9, 29.8 }, S.getData(), 0.1);
        // sin
        // assertArrayEquals(new double[] { 2.5, 0.6, 0.6, 2.1 }, S.getData(), 0.1);
        // no gravity
        assertArrayEquals(new double[] { 2.4, 1.1, 1.1, 2.6 }, S.getData(), 0.1);

        // these should be different.
        S = arena.getS(new double[] { Math.PI / 2, 0 });
        // assertArrayEquals(new double[] { 6.9, -4.9, -4.9, 20.1 }, S.getData(), 0.1);
        // no gravity
        assertArrayEquals(new double[] { 2.4, 1.1, 1.1, 2.6 }, S.getData(), 0.1);
        S = arena.getS(new double[] { -Math.PI / 2, 0 });
        // assertArrayEquals(new double[] { 112.6, 35.2, 35.2, 12.1 }, S.getData(), 0.1);
// no gravity
        assertArrayEquals(new double[] { 2.4, 1.1, 1.1, 2.6 }, S.getData(), 0.1);

        // different positions
        // this *should* know the difference in "cost" between pushing against gravity
        // and going with gravity. i think?
        assertEquals(38.47, arena.dist(new double[] { -Math.PI, 0 }, new double[] { -3 * Math.PI / 4, 0 }), 0.1);
        assertEquals(17.1, arena.dist(new double[] { -3 * Math.PI / 4, 0 }, new double[] { -Math.PI / 2, 0 }), 0.1);
        assertEquals(4.27, arena.dist(new double[] { -Math.PI / 2, 0 }, new double[] { -Math.PI / 4, 0 }), 0.1);
        assertEquals(4.27, arena.dist(new double[] { -Math.PI / 4, 0 }, new double[] { 0, 0 }), 0.1);
        assertEquals(0, arena.dist(new double[] { 0, 0 }, new double[] { 0, 0 }), 0.1);
        assertEquals(4.27, arena.dist(new double[] { 0, 0 }, new double[] { Math.PI / 4, 0 }), 0.1);
        assertEquals(17.1, arena.dist(new double[] { Math.PI / 4, 0 }, new double[] { Math.PI / 2, 0 }), 0.1);
        assertEquals(38.47, arena.dist(new double[] { Math.PI / 2, 0 }, new double[] { 3 * Math.PI / 4, 0 }), 0.1);
        assertEquals(68.39, arena.dist(new double[] { 3 * Math.PI / 4, 0 }, new double[] { Math.PI, 0 }), 0.1);

        // different speeds
        // this is stopping in the direction you're going
        assertEquals(0, arena.dist(new double[] { 0, 1 }, new double[] { 1, 0 }), 0.1);
        assertEquals(0, arena.dist(new double[] { 0, -1 }, new double[] { -1, 0 }), 0.1);
        // this is reverse, should be different
        assertEquals(0, arena.dist(new double[] { 0, 1 }, new double[] { -1, 0 }), 0.1);
        assertEquals(0, arena.dist(new double[] { 0, -1 }, new double[] { 1, 0 }), 0.1);

    }

    @Test
    void testSteer() {

    }

}
