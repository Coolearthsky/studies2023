package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.FrictionRotary1D;
import org.team100.lib.system.examples.NormalDoubleIntegratorRotary1D;
import org.team100.lib.system.examples.Pendulum1D;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.LinearSystem;

public class KTest {
    static final double kDelta = 0.001;
    static final double kDt = 0.02;

    /**
     * exactly from the LQR constructor, for comparison
     */
    public static Matrix<N1, N2> calculateK(
            Matrix<N2, N2> A,
            Matrix<N2, N1> B,
            Matrix<N2, N2> Q,
            Matrix<N1, N1> R,
            double dtSeconds) {
        var discABPair = Discretization.discretizeAB(A, B, dtSeconds);
        var discA = discABPair.getFirst();
        var discB = discABPair.getSecond();

        if (!StateSpaceUtil.isStabilizable(discA, discB)) {
            var builder = new StringBuilder("The system passed to the LQR is uncontrollable!\n\nA =\n");
            builder.append(discA.getStorage().toString())
                    .append("\nB =\n")
                    .append(discB.getStorage().toString())
                    .append('\n');
            throw new IllegalArgumentException(builder.toString());
        }

        var S = Drake.discreteAlgebraicRiccatiEquation(discA, discB, Q, R);

        // K = (BᵀSB + R)⁻¹BᵀSA
        Matrix<N1, N2> m_K = discB
                .transpose()
                .times(S)
                .times(discB)
                .plus(R)
                .solve(discB.transpose().times(S).times(discA));
        return m_K;
    }

    /**
     * verify that we're calculating K the same as LQR does.
     */
    @Test
    public void testK() {
        Nat<N2> states = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        Matrix<N2, N2> A = Matrix.mat(states, states).fill(0, 1, 0, 0);
        Matrix<N2, N1> B = Matrix.mat(states, inputs).fill(0, 1);
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        Matrix<N2, N2> Q = StateSpaceUtil.makeCostMatrix(stateTolerance);
        Matrix<N1, N1> R = StateSpaceUtil.makeCostMatrix(controlTolerance);
        Matrix<N1, N2> K = calculateK(A, B, Q, R, kDt);
        assertEquals(572.773, K.get(0, 0), kDelta);
        assertEquals(44.336, K.get(0, 1), kDelta);
    }

    /**
     * verify that we're calculating K the same as LQR does using the jacobian
     */
    @Test
    public void testFK() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        Matrix<N2, N1> x = VecBuilder.fill(0, 0);
        Matrix<N1, N1> u = VecBuilder.fill(0);

        NonlinearPlant<N2, N1, N2> plant = new NormalDoubleIntegratorRotary1D();
        LinearizedLQR<N2, N1, N2> angleController = new LinearizedLQR<>(plant, stateTolerance, controlTolerance);

        Matrix<N1, N2> K = angleController.calculateK(x, u, kDt);
        assertEquals(572.773, K.get(0, 0), kDelta);
        assertEquals(44.336, K.get(0, 1), kDelta);
    }

    /**
     * verify that we're calculating K the same as LQR does using the jacobian
     */
    @Test
    public void testFrictionFK() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        Matrix<N2, N1> x = VecBuilder.fill(0, 0);
        Matrix<N1, N1> u = VecBuilder.fill(0);

        NonlinearPlant<N2, N1, N2> plant = new FrictionRotary1D();
        LinearizedLQR<N2, N1, N2> angleController = new LinearizedLQR<>(plant, stateTolerance, controlTolerance);

        Matrix<N1, N2> K = angleController.calculateK(x, u, kDt);
        assertEquals(578.494, K.get(0, 0), kDelta);
        assertEquals(43.763, K.get(0, 1), kDelta);
    }

    /**
     * look at some values of K for the pendulum.
     */
    @Test
    public void testPendulumK() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        {
            Matrix<N2, N1> x = VecBuilder.fill(0, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);

            NonlinearPlant<N2, N1, N2> plant = new Pendulum1D();
            LinearizedLQR<N2, N1, N2> angleController = new LinearizedLQR<>(plant, stateTolerance, controlTolerance);

            Matrix<N1, N2> K = angleController.calculateK(x, u, kDt);
            // same as double integrator when gravity is max
            assertEquals(572.773, K.get(0, 0), kDelta);
            assertEquals(44.336, K.get(0, 1), kDelta);
        }
        {
            Matrix<N2, N1> x = VecBuilder.fill(Math.PI / 4, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);

            NonlinearPlant<N2, N1, N2> plant = new Pendulum1D();
            LinearizedLQR<N2, N1, N2> angleController = new LinearizedLQR<>(plant, stateTolerance, controlTolerance);

            Matrix<N1, N2> K = angleController.calculateK(x, u, kDt);

            assertEquals(573.425, K.get(0, 0), kDelta);// very slightly higher
            assertEquals(44.343, K.get(0, 1), kDelta);// very slightly higher
        }
        {
            Matrix<N2, N1> x = VecBuilder.fill(Math.PI / 2, 0);
            Matrix<N1, N1> u = VecBuilder.fill(0);

            NonlinearPlant<N2, N1, N2> plant = new Pendulum1D();
            LinearizedLQR<N2, N1, N2> angleController = new LinearizedLQR<>(plant, stateTolerance, controlTolerance);

            Matrix<N1, N2> K = angleController.calculateK(x, u, kDt);
            assertEquals(573.695, K.get(0, 0), kDelta); // a tiny bit higher
            assertEquals(44.346, K.get(0, 1), kDelta);// a tiny bit higher
        }
    }

    /**
     * what does the LQR version do
     */
    @Test
    public void testLQRK() {
        // double integrator
        Nat<N2> states = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        Nat<N2> outputs = Nat.N2();
        Matrix<N2, N2> A = Matrix.mat(states, states).fill(0, 1, 0, 0);
        Matrix<N2, N1> B = Matrix.mat(states, inputs).fill(0, 1);
        Matrix<N2, N2> C = Matrix.mat(outputs, states).fill(1, 0, 0, 1);
        Matrix<N2, N1> D = Matrix.mat(outputs, inputs).fill(0, 0);
        LinearSystem<N2, N1, N2> plant = new LinearSystem<>(A, B, C, D);
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<>(plant, stateTolerance,
                controlTolerance, kDt);
        Matrix<N1, N2> K = lqr.getK();
        assertEquals(572.773, K.get(0, 0), kDelta);
        assertEquals(44.336, K.get(0, 1), kDelta);
    }

    /**
     * what does the LQR version do
     */
    @Test
    public void testFrictionLQRK() {
        // double integrator
        Nat<N2> states = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        Nat<N2> outputs = Nat.N2();
        // note the last term here
        Matrix<N2, N2> A = Matrix.mat(states, states).fill(0, 1, 0, -1);
        Matrix<N2, N1> B = Matrix.mat(states, inputs).fill(0, 1);
        Matrix<N2, N2> C = Matrix.mat(outputs, states).fill(1, 0, 0, 1);
        Matrix<N2, N1> D = Matrix.mat(outputs, inputs).fill(0, 0);
        LinearSystem<N2, N1, N2> plant = new LinearSystem<>(A, B, C, D);
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<>(plant, stateTolerance,
                controlTolerance, kDt);
        Matrix<N1, N2> K = lqr.getK();
        assertEquals(578.494, K.get(0, 0), kDelta);
        // less velocity feedback needed because of damping
        assertEquals(43.763, K.get(0, 1), kDelta);
    }

}
