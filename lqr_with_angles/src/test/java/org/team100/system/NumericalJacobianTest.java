package org.team100.system;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.BiFunction;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalJacobian;

public class NumericalJacobianTest {
    private static final double kDelta = 0.0001;

    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u
     * 
     * so the u jacobian should just be [0, 1]
     */
    Matrix<N2, N1> newton1d(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        //double p = xmat.get(0, 0);
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u;
        return VecBuilder.fill(pdot, vdot);
    }

    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u - cos(p)
     * 
     * so vdot itself depends on p but it is still linear in u.
     */
    Matrix<N2, N1> pendulum(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        double p = xmat.get(0, 0);
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u - Math.cos(p);
        return VecBuilder.fill(pdot, vdot);
    }

    @Test
    public void testNewton() {
        Nat<N2> rows = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = this::newton1d;
        Matrix<N2, N1> x = new Matrix<>(Nat.N2(), Nat.N1()); // zero
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1()); // zero
        // calculates the continuous B linearized around x and u.
        // so this is linearizing around x=0 and u=0
        // but B is constant anyway.
        {
            Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(rows, inputs, f, x, u);
            assertEquals(0, B.get(0, 0), kDelta);
            assertEquals(1, B.get(1, 0), kDelta);
        }
        x.set(0, 0, 1); // different position
        {
            Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(rows, inputs, f, x, u);
            assertEquals(0, B.get(0, 0), kDelta);
            assertEquals(1, B.get(1, 0), kDelta);
        }
    }

    @Test
    public void testPendulum() {
        Nat<N2> rows = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = this::pendulum;
        Matrix<N2, N1> x = new Matrix<>(Nat.N2(), Nat.N1()); // zero
        Matrix<N1, N1> u = new Matrix<>(Nat.N1(), Nat.N1()); // zero
        // calculates the continuous B linearized around x and u.
        // so this is linearizing around x=0 and u=0
        // but B is constant anyway.
        {
            Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(rows, inputs, f, x, u);
            assertEquals(0, B.get(0, 0), kDelta);
            assertEquals(1, B.get(1, 0), kDelta);
        }
        x.set(0, 0, 1); // different position
        {
            Matrix<N2, N1> B = NumericalJacobian.numericalJacobianU(rows, inputs, f, x, u);
            assertEquals(0, B.get(0, 0), kDelta);
            assertEquals(1, B.get(1, 0), kDelta);
        }
    }

}
