package org.team100.lib.system.examples;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional double-integrator with friction force proportional to
 * velocity.
 */
public abstract class FrictionCartesian1D extends CartesianPlant1D {
    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u
     * 
     * the x jacobian should be constant [0 1 0 -1]
     * the u jacobian should be constant [0 1]
     */
    @Override
    public Matrix<N2, N1> f(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u - v;
        return VecBuilder.fill(pdot, vdot);
    }

}
