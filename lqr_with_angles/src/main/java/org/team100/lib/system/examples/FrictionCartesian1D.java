package org.team100.lib.system.examples;

import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional double-integrator with friction force proportional to
 * velocity.
 */
public class FrictionCartesian1D extends CartesianPlant1D {
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

    @Override
    public Matrix<N2, N1> stdev() {
        return VecBuilder.fill(0.015, 0.17);
    }

    @Override
    public Sensor<N2, N1, N2> newFull() {
        return new FullSensor() {
            public Matrix<N2, N1> stdev() {
                return VecBuilder.fill(0.01, 0.1);
            }
        };
    }

    @Override
    public Sensor<N2, N1, N1> newPosition() {
        return new PositionSensor() {
            public Matrix<N1, N1> stdev() {
                return VecBuilder.fill(0.01);
            }
        };
    }

    @Override
    public Sensor<N2, N1, N1> newVelocity() {
        return new VelocitySensor() {
            public Matrix<N1, N1> stdev() {
                return VecBuilder.fill(0.1);
            }
        };
    }

}
