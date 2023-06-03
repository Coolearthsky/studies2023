package org.team100.lib.system.examples;

import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Base class for one-dimensional rotational plants. */
public abstract class RotaryPlant1D implements NonlinearPlant<N2, N1, N2>  {
    private final double positionMeasurementStdev;
    private final double velocityMeasurementStdev;
    private final double positionStateStdev;
    private final double velocityStateStdev;
    private final Sensor<N2, N1, N2> full;
    private final Sensor<N2, N1, N1> position;
    private final Sensor<N2, N1, N1> velocity;

    public class FullSensor implements Sensor<N2, N1, N2> {
        public Matrix<N2, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
            return x;
        }

        public Matrix<N2, N1> yResidual(Matrix<N2, N1> a, Matrix<N2, N1> b) {
            return AngleStatistics.angleResidual(a, b, 0);
        }

        public Matrix<N2, N1> stdev() {
            return VecBuilder.fill(positionMeasurementStdev, velocityMeasurementStdev);
        }

        public Nat<N2> rows() {
            return Nat.N2();
        }
    }

    public class PositionSensor implements Sensor<N2, N1, N1> {
        public Matrix<N1, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
            return VecBuilder.fill(x.get(0, 0));
        }

        public Matrix<N1, N1> yResidual(Matrix<N1, N1> a, Matrix<N1, N1> b) {
            return AngleStatistics.angleResidual(a, b, 0);
        }

        public Matrix<N1, N1> stdev() {
            return VecBuilder.fill(positionMeasurementStdev);
        }

        @Override
        public Nat<N1> rows() {
            return Nat.N1();
        }
    }

    public class VelocitySensor implements Sensor<N2, N1, N1> {
        public Matrix<N1, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
            return VecBuilder.fill(x.get(1, 0));
        }

        public Matrix<N1, N1> yResidual(Matrix<N1, N1> a, Matrix<N1, N1> b) {
            return a.minus(b);
        }

        public Matrix<N1, N1> stdev() {
            return VecBuilder.fill(velocityMeasurementStdev);
        }

        @Override
        public Nat<N1> rows() {
            return Nat.N1();
        }
    }

    // TODO: remove these stdevs, put them in the sensor only
    public RotaryPlant1D(double positionMeasurementStdev, double velocityMeasurementStdev,
            double positionStateStdev, double velocityStateStdev) {
        this.positionMeasurementStdev = positionMeasurementStdev;
        this.velocityMeasurementStdev = velocityMeasurementStdev;
        this.positionStateStdev = positionStateStdev;
        this.velocityStateStdev = velocityStateStdev;
        full = new FullSensor();
        position = new PositionSensor();
        velocity = new VelocitySensor();
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

    public Sensor<N2, N1, N1> velocity() {
        return velocity;
    }

    public Sensor<N2, N1, N2> full() {
        return full;
    }

    public Matrix<N2, N1> stdev() {
        return VecBuilder.fill(positionStateStdev, velocityStateStdev);
    }
    
    public Matrix<N2, N1> xNormalize(Matrix<N2, N1> xmat) {
        Matrix<N2,N1> x = xmat.copy();
        x.set(0,0, MathUtil.angleModulus(x.get(0,0)));
        return x;
    }

    public Matrix<N1, N1> limit(Matrix<N1, N1> u) {
        return StateSpaceUtil.desaturateInputVector(u, 12.0);
    }

    public Nat<N2> states() {
        return Nat.N2();
    }

    public Nat<N1> inputs() {
        return Nat.N1();
    }

    public Nat<N2> outputs() {
        return Nat.N2();
    }

}
