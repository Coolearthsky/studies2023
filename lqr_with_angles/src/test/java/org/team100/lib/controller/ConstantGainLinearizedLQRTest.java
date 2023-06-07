package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.FrictionCartesian1D;
import org.team100.lib.system.examples.Pendulum1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ConstantGainLinearizedLQRTest {
    /** K dependence on x is weak, not worth recalculating K all the time. */
    @Test
    public void testPendulumK() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        NonlinearPlant<N2, N1, N2> plant = new Pendulum1D();
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(
                plant, stateTolerance, controlTolerance, 0.01);
        // full gravity
        RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(0, 0), new Matrix<>(Nat.N2(), Nat.N2()));
        Matrix<N1, N1> u = VecBuilder.fill(0);
        Matrix<N1, N2> K = controller.calculateK(x, u, 0.01);
        assertArrayEquals(new double[] { 818.6, 57.5 }, K.getData(), 0.1);
        // x = VecBuilder.fill(Math.PI / 2, 0); // no gravity
        x = new RandomVector<>(VecBuilder.fill(Math.PI / 2, 0), new Matrix<>(Nat.N2(), Nat.N2()));
        K = controller.calculateK(x, u, 0.01);
        // this is *very* slightly different.
        assertArrayEquals(new double[] { 819.6, 57.5 }, K.getData(), 0.1);
    }

    /** K dependence on x is zero. */
    @Test
    public void testFrictionK() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        NonlinearPlant<N2, N1, N2> plant = new FrictionCartesian1D();
        ConstantGainLinearizedLQR<N2, N1, N2> controller = new ConstantGainLinearizedLQR<>(
                plant, stateTolerance, controlTolerance, 0.01);
        // Matrix<N2, N1> x = VecBuilder.fill(0, 0); // no friction force
        RandomVector<N2> x = new RandomVector<>(VecBuilder.fill(0, 0), new Matrix<>(Nat.N2(), Nat.N2()));
        Matrix<N1, N1> u = VecBuilder.fill(0);
        Matrix<N1, N2> K = controller.calculateK(x, u, 0.01);
        assertArrayEquals(new double[] { 822.7, 56.8 }, K.getData(), 0.1);
        // x = VecBuilder.fill(0, 10); // some friction force
        x = new RandomVector<>(VecBuilder.fill(0, 10), new Matrix<>(Nat.N2(), Nat.N2()));
        K = controller.calculateK(x, u, 0.01);
        // same K
        assertArrayEquals(new double[] { 822.7, 56.8 }, K.getData(), 0.1);
    }
}
