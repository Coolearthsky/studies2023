package org.team100.lib.system.examples;

import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional pendulum with gravity. Angle is measured from horizontal.
 * State includes velocity and position, input is acceleration, output is
 * position.
 */
public class Pendulum1D implements NonlinearPlant<N2, N1, N2> {
    private final Sensor<N2, N1, N2> full;
    private final Sensor<N2, N1, N1> position;

    public class FullSensor implements Sensor<N2,N1,N2> {
        public Matrix<N2, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
            return x;
        }
        public Matrix<N2, N1> yResidual(Matrix<N2, N1> a, Matrix<N2, N1> b) {
            return AngleStatistics.angleResidual(a, b, 0);
        }
    }
    public class PositionSensor implements Sensor<N2,N1,N1> {
        public Matrix<N1, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
            return VecBuilder.fill(x.get(0, 0));
        }
        public Matrix<N1, N1> yResidual(Matrix<N1, N1> a, Matrix<N1, N1> b) {
            return AngleStatistics.angleResidual(a, b, 0);
        }
    }


    public Pendulum1D() {
        full = new FullSensor();
        position = new PositionSensor();
    }

    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u - cos(p)
     * 
     * so vdot itself depends on p but it is still linear in u.
     */
    @Override
    public Matrix<N2, N1> f(Matrix<N2, N1> xmat, Matrix<N1, N1> umat) {
        double p = xmat.get(0, 0);
        double v = xmat.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u - Math.cos(p);
        return VecBuilder.fill(pdot, vdot);
    }

    /**
     * State dimension 0 is an angle.
     */
    @Override
    public Matrix<N2, N1> xResidual(Matrix<N2, N1> a, Matrix<N2, N1> b) {
        return AngleStatistics.angleResidual(a, b, 0);
    }

    @Override
    public Matrix<N2, N1> xAdd(Matrix<N2, N1> a, Matrix<N2, N1> b) {
        return AngleStatistics.angleAdd(a, b, 0);
    }

    /**
     * Measure position.
     */
    public Sensor<N2, N1, N1> position() {
        return position;
    }

    public Sensor<N2, N1, N2> full() {
        return full;
    }
}
