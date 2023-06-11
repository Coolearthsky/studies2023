package org.team100.lib.system.examples;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional double integrator, represents frictionless newtonian motion.
 * State includes velocity and position, input is acceleration, output is
 * position.
 */
public class DoubleIntegratorCartesian1D extends CartesianPlant1D {
    @Override
    public RandomVector<N2> f(RandomVector<N2> xmat, Matrix<N1, N1> umat) {
        double v = xmat.x.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u;
        // TODO: handle P correctly
        return new RandomVector<>(VecBuilder.fill(pdot, vdot), xmat.P);
    }

    @Override
    public Matrix<N1, N1> finvWrtU(RandomVector<N2> x, RandomVector<N2> xdot) {
        double a = xdot.x.get(1,0);
        return VecBuilder.fill(a);
    }

    @Override
    public RandomVector<N2> finvWrtX(RandomVector<N2> xdot, Matrix<N1, N1> u) {
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        xx.set(1, 0, xdot.x.get(0, 0));
        Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
        xP.set(0, 0, 1e9); // "don't know" variance
        xP.set(1, 1, xdot.P.get(0, 0));
        return new RandomVector<>(xx, xP); 
    }
}
