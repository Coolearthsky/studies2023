package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.system.examples.DoubleIntegrator1D;
import org.team100.lib.system.examples.Pendulum1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalJacobian;

public class NumericalJacobianTest {
    private static final double kDelta = 0.0001;

    /**
     * A = [0 1 0 0] constant
     */
    @Test
    public void testDoubleIntegratorA() {
        Nat<N2> rows = Nat.N2();
        Nat<N2> states = Nat.N2();
        NonlinearPlant<N2, N1, N2> system = new DoubleIntegrator1D(0, 0, 0, 0);
        {
            // at zero
            Matrix<N2, N1> x = VecBuilder.fill(0, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = NumericalJacobian.numericalJacobianX(rows, states, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, A.get(0, 0), kDelta),
                    () -> assertEquals(1, A.get(0, 1), kDelta),
                    () -> assertEquals(0, A.get(1, 0), kDelta),
                    () -> assertEquals(0, A.get(1, 1), kDelta));
        }
        {
            // different position, same A
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = NumericalJacobian.numericalJacobianX(rows, states, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, A.get(0, 0), kDelta),
                    () -> assertEquals(1, A.get(0, 1), kDelta),
                    () -> assertEquals(0, A.get(1, 0), kDelta),
                    () -> assertEquals(0, A.get(1, 1), kDelta));
        }
        {
            // different u, same A
            Matrix<N2, N1> x = VecBuilder.fill(0, 0);
            Matrix<N1, N1> u = VecBuilder.fill(10000);
            Matrix<N2, N2> A = NumericalJacobian.numericalJacobianX(rows, states, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, A.get(0, 0), kDelta),
                    () -> assertEquals(1, A.get(0, 1), kDelta),
                    () -> assertEquals(0, A.get(1, 0), kDelta),
                    () -> assertEquals(0, A.get(1, 1), kDelta));
        }
    }

    /**
     * B = [0 1] constant
     */
    @Test
    public void testDoubleIntegratorB() {
        Nat<N2> rows = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        NonlinearPlant<N2, N1, N2> system = new DoubleIntegrator1D(0, 0, 0, 0);
        {
            // at zero
            Matrix<N2, N1> x = VecBuilder.fill(0, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(rows, inputs, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, B.get(0, 0), kDelta),
                    () -> assertEquals(1, B.get(1, 0), kDelta));
        }
        {
            // different position, same B
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(rows, inputs, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, B.get(0, 0), kDelta),
                    () -> assertEquals(1, B.get(1, 0), kDelta));
        }
        {
            // different u, same B
            Matrix<N2, N1> x = VecBuilder.fill(0, 0);
            Matrix<N1, N1> u = VecBuilder.fill(10000);
            Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(rows, inputs, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, B.get(0, 0), kDelta),
                    () -> assertEquals(1, B.get(1, 0), kDelta));
        }
    }

    /**
     * A = [0 1 sin(p) 0]
     */
    @Test
    public void testPendulumA() {
        Nat<N2> rows = Nat.N2();
        Nat<N2> states = Nat.N2();
        NonlinearPlant<N2, N1, N2> system = new Pendulum1D(0, 0, 0, 0);
        {
            // at zero degrees the gravity force doesn't change much with position
            // so the jacobian is zero
            Matrix<N2, N1> x = VecBuilder.fill(0, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = NumericalJacobian.numericalJacobianX(rows, states, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, A.get(0, 0), kDelta),
                    () -> assertEquals(1, A.get(0, 1), kDelta),
                    () -> assertEquals(0, A.get(1, 0), kDelta),
                    () -> assertEquals(0, A.get(1, 1), kDelta));
        }
        {
            // at 30 degrees the gravity force changes a little with position
            Matrix<N2, N1> x = VecBuilder.fill(Math.PI / 6, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = NumericalJacobian.numericalJacobianX(rows, states, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, A.get(0, 0), kDelta),
                    () -> assertEquals(1, A.get(0, 1), kDelta),
                    () -> assertEquals(0.5, A.get(1, 0), kDelta),
                    () -> assertEquals(0, A.get(1, 1), kDelta));
        }
        {
            // at 60 degrees the gravity force changes a lot with position
            Matrix<N2, N1> x = VecBuilder.fill(Math.PI / 3, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = NumericalJacobian.numericalJacobianX(rows, states, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, A.get(0, 0), kDelta),
                    () -> assertEquals(1, A.get(0, 1), kDelta),
                    () -> assertEquals(0.866, A.get(1, 0), kDelta),
                    () -> assertEquals(0, A.get(1, 1), kDelta));
        }
        {
            // at 90 degrees the gravity force is about proportional with position
            Matrix<N2, N1> x = VecBuilder.fill(Math.PI / 2, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N2> A = NumericalJacobian.numericalJacobianX(rows, states, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, A.get(0, 0), kDelta),
                    () -> assertEquals(1, A.get(0, 1), kDelta),
                    () -> assertEquals(1, A.get(1, 0), kDelta),
                    () -> assertEquals(0, A.get(1, 1), kDelta));
        }
    }

    /**
     * B = [0 1] constant because while the model is nonlinear the control response
     * is linear.
     */
    @Test
    public void testPendulumB() {
        Nat<N2> rows = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        NonlinearPlant<N2, N1, N2> system = new Pendulum1D(0, 0, 0, 0);
        {
            // at zero
            Matrix<N2, N1> x = VecBuilder.fill(0, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(rows, inputs, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, B.get(0, 0), kDelta),
                    () -> assertEquals(1, B.get(1, 0), kDelta));
        }
        {
            // different position, same B
            Matrix<N2, N1> x = VecBuilder.fill(1, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);
            Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(rows, inputs, system::f, x, u);
            assertAll(
                    () -> assertEquals(0, B.get(0, 0), kDelta),
                    () -> assertEquals(1, B.get(1, 0), kDelta));
        }
    }
}
