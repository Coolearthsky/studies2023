package org.team100.lib.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.BiFunction;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.ShootingSolver.Solution;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.NumericalIntegration;

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
    void testAngleSum() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1);
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(1, 1);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            Matrix<N2, N1> x2x1 = x2.minus(x1);
            Matrix<N2, N1> x2minX2 = x2.minus(minX2);
            Matrix<N2, N1> x2maxX2 = x2.minus(maxX2);
            // 45 + 45 + 90 = pi
            assertEquals(Math.PI, s.angleSum(x2x1, x2minX2, x2maxX2));
        }
    }

    @Test
    void testAngleSum3d() {
        // 3d because i can visualize it; the actual case is 4d.
        ShootingSolver<N3, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1);
        {
            // this plane is x+y+z=1
            // so the closest point to 0,0,0 is 1/sqrt(3) away
            // along the inner diagonal. a cube of size 1 has
            // inner diagonal of sqrt(3) so this little cube has
            // size 1/3.
            Matrix<N3, N1> x1 = VecBuilder.fill(1, 0, 0);
            Matrix<N3, N1> x2 = VecBuilder.fill(0.333, 0.333, 0.333);
            Matrix<N3, N1> minX2 = VecBuilder.fill(0, 1, 0);
            Matrix<N3, N1> maxX2 = VecBuilder.fill(0, 0, 1);
            Matrix<N3, N1> x2x1 = x2.minus(x1);
            Matrix<N3, N1> x2minX2 = x2.minus(minX2);
            Matrix<N3, N1> x2maxX2 = x2.minus(maxX2);
            assertEquals(2.0 * Math.PI, s.angleSum(x2x1, x2minX2, x2maxX2), 0.001);
        }
    }

    @Test
    void testInside2d() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1);
        {
            // far outside
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(1, 1);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            assertFalse(s.inside(x1, x2, minX2, maxX2));
        }
        {
            // far inside
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(0.25, 0.25);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            assertTrue(s.inside(x1, x2, minX2, maxX2));
        }
        {
            // on the boundary, should be inside
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N2, N1> x2 = VecBuilder.fill(0.5, 0.5);
            Matrix<N2, N1> minX2 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> maxX2 = VecBuilder.fill(1, 0);
            assertTrue(s.inside(x1, x2, minX2, maxX2));
        }
    }

    @Test
    void testInside3d() {
        // 3d because i can visualize it; the actual case is 4d.
        ShootingSolver<N3, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 0.1);
        {
            // far outside
            Matrix<N3, N1> x1 = VecBuilder.fill(0, 0, 0);
            Matrix<N3, N1> x2 = VecBuilder.fill(1, 1, 1);
            Matrix<N3, N1> minX2 = VecBuilder.fill(0, 1, 0);
            Matrix<N3, N1> maxX2 = VecBuilder.fill(1, 0, 0);
            assertFalse(s.inside(x1, x2, minX2, maxX2));
        }
        {
            // in the middle of the triangle
            Matrix<N3, N1> x1 = VecBuilder.fill(1, 0, 0);
            Matrix<N3, N1> x2 = VecBuilder.fill(0.333, 0.333, 0.333);
            Matrix<N3, N1> minX2 = VecBuilder.fill(0, 1, 0);
            Matrix<N3, N1> maxX2 = VecBuilder.fill(0, 0, 1);
            assertTrue(s.inside(x1, x2, minX2, maxX2));
        }
        {
            // near but not actually on the plane
            Matrix<N3, N1> x1 = VecBuilder.fill(1, 0, 0);
            Matrix<N3, N1> x2 = VecBuilder.fill(0.4, 0.4, 0.4);
            Matrix<N3, N1> minX2 = VecBuilder.fill(0, 1, 0);
            Matrix<N3, N1> maxX2 = VecBuilder.fill(0, 0, 1);
            assertFalse(s.inside(x1, x2, minX2, maxX2));
        }
    }

    @Test
    void testRK4MotionlessStart() {
        // double-integrator
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (x, u) -> {
            return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
        };
        double dt = 1;
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
            Matrix<N1, N1> maxU = VecBuilder.fill(1);
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, maxU, dt);
            // x = x0 + 0.5 u t^2 = 0.5
            // v = v0 + u t = 1
            assertArrayEquals(new double[] { 0.5, 1 }, maxX2.getData(), 0.001);
        }
    }

    @Test
    void testRK4MovingStart() {
        // double-integrator
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (x, u) -> {
            return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
        };
        Matrix<N2, N1> x1 = VecBuilder.fill(0, 1);
        {
            Matrix<N1, N1> u = VecBuilder.fill(-1);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 - 0.5 = 0.5
            // v = v0 + u = 1 - 1 = 0
            assertArrayEquals(new double[] { 0.5, 0 }, maxX2.getData(), 0.001);
        }
        {
            Matrix<N1, N1> u = VecBuilder.fill(-0.5);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 - 0.5 = 0.5
            // v = v0 + u = 1 - 1 = 0
            assertArrayEquals(new double[] { 0.75, 0.5 }, maxX2.getData(), 0.001);
        }
        {
            Matrix<N1, N1> u = VecBuilder.fill(0);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t = 1
            // v = v0 = 1
            assertArrayEquals(new double[] { 1, 1 }, maxX2.getData(), 0.001);
        }
        {
            Matrix<N1, N1> u = VecBuilder.fill(0.5);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 + 0.5 = 1.5
            // v = v0 + u = 1 + 1 = 2
            assertArrayEquals(new double[] { 1.25, 1.5 }, maxX2.getData(), 0.001);
        }
        {
            Matrix<N1, N1> u = VecBuilder.fill(1);
            double dt = 1;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 + 0.5 = 1.5
            // v = v0 + u = 1 + 1 = 2
            assertArrayEquals(new double[] { 1.5, 2 }, maxX2.getData(), 0.001);
        }
        {
            // in the middle somewhere
            Matrix<N1, N1> u = VecBuilder.fill(0.5);
            double dt = 0.5;
            Matrix<N2, N1> maxX2 = NumericalIntegration.rk4(f, x1, u, dt);
            // x = x0 + v t + 0.5 u t ^ 2 = 1 + 0.5 = 1.5
            // v = v0 + u = 1 + 1 = 2
            // this is the solution my solver should find below
            assertArrayEquals(new double[] { 0.5625, 1.25 }, maxX2.getData(), 0.001);
        }
    }

    @Test
    void testPossibleMotionlessStart() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 1);

        // double-integrator
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (x, u) -> {
            return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
        };
        // motionless start makes the feasible region a line
        Matrix<N2, N1> x1 = VecBuilder.fill(0, 0);
        {
            // end = start
            Matrix<N2, N1> x2 = VecBuilder.fill(0, 0);
            assertTrue(s.possible(Nat.N2(), Nat.N1(), f, x1, x2));
        }
        {
            // end = minU
            Matrix<N2, N1> x2 = VecBuilder.fill(0.5, 1);
            assertTrue(s.possible(Nat.N2(), Nat.N1(), f, x1, x2));
        }
        {
            // end = maxU
            Matrix<N2, N1> x2 = VecBuilder.fill(-0.5, -1);
            assertTrue(s.possible(Nat.N2(), Nat.N1(), f, x1, x2));
        }
        {
            // end = between start and maxU
            Matrix<N2, N1> x2 = VecBuilder.fill(0.25, 0.5);
            assertTrue(s.possible(Nat.N2(), Nat.N1(), f, x1, x2));
        }
    }

    @Test
    void testPossibleMovingStart() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 1);

        // double-integrator
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (x, u) -> {
            return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
        };
        // motionless start makes the feasible region a line
        Matrix<N2, N1> x1 = VecBuilder.fill(0, 1);
        {
            // end = start
            Matrix<N2, N1> x2 = VecBuilder.fill(0, 1);
            assertTrue(s.possible(Nat.N2(), Nat.N1(), f, x1, x2));
        }
        {
            // end = minU
            Matrix<N2, N1> x2 = VecBuilder.fill(0.5, 0);
            assertTrue(s.possible(Nat.N2(), Nat.N1(), f, x1, x2));
        }
        {
            // end = maxU
            Matrix<N2, N1> x2 = VecBuilder.fill(1.5, 2);
            assertTrue(s.possible(Nat.N2(), Nat.N1(), f, x1, x2));
        }
        {
            // end = within the triangle
            Matrix<N2, N1> x2 = VecBuilder.fill(0.5, 1);
            assertTrue(s.possible(Nat.N2(), Nat.N1(), f, x1, x2));
        }
    }

    @Test
    void testSolve() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 1);

        // double-integrator
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (x, u) -> {
            return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
        };
        // motionless start makes the feasible region a line
        Matrix<N2, N1> x1 = VecBuilder.fill(0, 1);
        {
            // end = start
            Matrix<N2, N1> x2 = VecBuilder.fill(0, 1);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2);
            assertNotNull(sol);
            // u is irrelevant
            assertEquals(0, sol.dt, 0.001);
        }
        {
            // end = minU
            Matrix<N2, N1> x2 = VecBuilder.fill(0.5, 0);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2);
            assertNotNull(sol);
            assertEquals(-1, sol.u.get(0, 0), 0.001);
            assertEquals(1, sol.dt, 0.001);
        }
        {
            // end = maxU
            Matrix<N2, N1> x2 = VecBuilder.fill(1.5, 2);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2);
            assertNotNull(sol);
            assertEquals(1, sol.u.get(0, 0), 0.001);
            assertEquals(1, sol.dt, 0.001);
        }
        {
            // end = within the triangle (from above RK4 test)
            Matrix<N2, N1> x2 = VecBuilder.fill(0.5625, 1.25);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2);
            assertNotNull(sol);
            assertEquals(0.5, sol.u.get(0, 0), 0.001);
            assertEquals(0.5, sol.dt, 0.001);
        }
        {
            // end = very infeasible
            Matrix<N2, N1> x2 = VecBuilder.fill(-10, 10);
            ShootingSolver<N2, N1>.Solution sol = s.solve(Nat.N2(), Nat.N1(), f, x1, x2);
            assertNull(sol);
        }
    }

    @Test
    void testNear() {
        ShootingSolver<N2, N1> s = new ShootingSolver<>(VecBuilder.fill(1), 1);
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> x2 = VecBuilder.fill(0, 1);
            assertTrue(s.near(x1, x2));
        }
        {
            Matrix<N2, N1> x1 = VecBuilder.fill(0, 1);
            Matrix<N2, N1> x2 = VecBuilder.fill(0, 1.1);
            assertFalse(s.near(x1, x2));
        }
    }
}
