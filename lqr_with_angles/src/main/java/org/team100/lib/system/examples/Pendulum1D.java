package org.team100.lib.system.examples;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional pendulum with gravity. Angle is measured from horizontal.
 * State includes velocity and position, input is acceleration, output is
 * position.
 */
public class Pendulum1D extends RotaryPlant1D {
    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u - cos(p)
     * 
     * so vdot itself depends on p but it is still linear in u.
     */
    @Override
    public RandomVector<N2> f(RandomVector<N2> xmat, Matrix<N1, N1> umat) {
        double p = xmat.x.get(0, 0);
        double v = xmat.x.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u - Math.cos(p);
        // TODO: handle P correctly
        return new RandomVector<>(VecBuilder.fill(pdot, vdot), xmat.P);
    }
    @Override
    public Matrix<N1, N1> finv(RandomVector<N2> x, RandomVector<N2> xdot) {
        double a = xdot.x.get(1, 0);
        double p = x.x.get(0, 0);
        return VecBuilder.fill(a + Math.cos(p));
    }
}
