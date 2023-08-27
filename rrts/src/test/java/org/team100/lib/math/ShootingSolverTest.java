package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ShootingSolverTest {
    @Test
    void testNorm() {
        // Frobenius norm does what i think it does
        Matrix<N2, N1> x = VecBuilder.fill(3, 4);
        assertEquals(5, x.normF(), 0.001);
    }

    @Test
    void testDot() {
        // matrix multiplication does what i think it does
        {
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N2, N1> y = VecBuilder.fill(0, 1);
            double dot = x.transpose().times(y).get(0, 0);
            assertEquals(0, dot, 0.001);
            double xNorm = x.normF();
            double yNorm = y.normF();
            double angleRad = Math.acos(dot / (xNorm * yNorm));
            assertEquals(Math.PI / 2, angleRad, 0.001);
        }
        {
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N2, N1> y = VecBuilder.fill(1, 1);
            double dot = x.transpose().times(y).get(0, 0);
            assertEquals(1, dot, 0.001);
            double xNorm = x.normF();
            double yNorm = y.normF();
            double angleRad = Math.acos(dot / (xNorm * yNorm));
            assertEquals(Math.PI / 4, angleRad, 0.001);
        }
    }

    @Test
    void testAngle() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1);
        {
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N2, N1> y = VecBuilder.fill(0, 1);
            assertEquals(Math.PI / 2, s.angleRad(x, y), 0.001);
        }
        {
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N2, N1> y = VecBuilder.fill(1, 1);
            assertEquals(Math.PI / 4, s.angleRad(x, y), 0.001);
        }
    }

    @Test
    void testInside2d() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1);
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(1, 1);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            assertFalse(s.inside(x1, x2, minX2, maxX2));
        }
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(0.25, 0.25);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            assertTrue(s.inside(x1, x2, minX2, maxX2));
        }

    }

}
