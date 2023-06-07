package org.team100.lib.system.examples;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional double integrator, represents frictionless newtonian motion.
 * State includes velocity and position, input is acceleration, output is
 * position.
 * 
 * In this case, we're modeling rotation, i.e. a wheel.
 */
public abstract class DoubleIntegratorRotary1D extends RotaryPlant1D {
    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u
     * 
     * the x jacobian should be constant [0 1 0 0]
     * the u jacobian should be constant [0, 1]
     */
    @Override
    public RandomVector<N2> f(RandomVector<N2> xmat, Matrix<N1, N1> umat) {
        double v = xmat.x.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u;
        // TODO: handle P correctly
        return new RandomVector<>(VecBuilder.fill(pdot, vdot), xmat.P);
    }
}
