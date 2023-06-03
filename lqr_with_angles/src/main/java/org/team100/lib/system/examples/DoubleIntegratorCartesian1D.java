package org.team100.lib.system.examples;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional double integrator, represents frictionless newtonian motion.
 * State includes velocity and position, input is acceleration, output is
 * position.
 */
public class DoubleIntegratorCartesian1D extends CartesianPlant1D {

    public DoubleIntegratorCartesian1D(double positionMeasurementStdev, double velocityMeasurementStdev,
            double positionStateStdev, double velocityStateStdev) {
        super(positionMeasurementStdev, velocityMeasurementStdev, positionStateStdev, velocityStateStdev);
    }

    @Override
    public Matrix<N2, N1> f(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u;
        return VecBuilder.fill(pdot, vdot);
    }

}
