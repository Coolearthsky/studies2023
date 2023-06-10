package org.team100.lib.system.examples;

import org.team100.lib.math.RandomVector;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional double-integrator with friction force proportional to
 * velocity.
 * 
 * In this case, we're modeling rotation, e.g. a wheel.
 */
public class FrictionRotary1D extends RotaryPlant1D {
    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u
     * 
     * the x jacobian should be constant [0 1 0 -1]
     * the u jacobian should be constant [0 1]
     */
    public RandomVector<N2> f(RandomVector<N2> xmat, Matrix<N1, N1> umat) {
        double v = xmat.x.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u - v;
        // return VecBuilder.fill(pdot, vdot);
        // TODO: handle P correctly
        return new RandomVector<>(VecBuilder.fill(pdot, vdot), xmat.P);

    }

    // public Matrix<N2, N1> stdev() {
    //     return VecBuilder.fill(0.015, 0.17);
    // }

    public Sensor<N2, N1, N2> newFull() {
        return new FullSensor() {
            // public Matrix<N2, N1> stdev() {
            //     return VecBuilder.fill(0.01, 0.1);
            // }
        };
    }

    public Sensor<N2, N1, N1> newPosition() {
        return new PositionSensor() {
            // public Matrix<N1, N1> stdev() {
            //     return VecBuilder.fill(0.01);
            // }
        };
    }

    public Sensor<N2, N1, N1> newVelocity() {
        return new VelocitySensor() {
            // public Matrix<N1, N1> stdev() {
            //     return VecBuilder.fill(0.1);
            // }
        };
    }
}
