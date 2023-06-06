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
public abstract class RotaryPlant1D implements NonlinearPlant<N2, N1, N2> {
    private final Sensor<N2, N1, N2> full;
    private final Sensor<N2, N1, N1> position;
    private final Sensor<N2, N1, N1> velocity;

    public abstract class FullSensor implements Sensor<N2, N1, N2> {
        public Matrix<N2, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
            return x;
        }

        public Matrix<N2, N1> yResidual(Matrix<N2, N1> a, Matrix<N2, N1> b) {
            return AngleStatistics.angleResidual(a, b, 0);
        }

        public Nat<N2> rows() {
            return Nat.N2();
        }
    }

    public abstract class PositionSensor implements Sensor<N2, N1, N1> {
        public Matrix<N1, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
            return VecBuilder.fill(x.get(0, 0));
        }

        public Matrix<N1, N1> yResidual(Matrix<N1, N1> a, Matrix<N1, N1> b) {
            return AngleStatistics.angleResidual(a, b, 0);
        }

        public Nat<N1> rows() {
            return Nat.N1();
        }
    }

    public abstract class VelocitySensor implements Sensor<N2, N1, N1> {
        public Matrix<N1, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
            return VecBuilder.fill(x.get(1, 0));
        }

        public Matrix<N1, N1> yResidual(Matrix<N1, N1> a, Matrix<N1, N1> b) {
            return a.minus(b);
        }

        public Nat<N1> rows() {
            return Nat.N1();
        }
    }

    public RotaryPlant1D() {
        full = newFull();
        position = newPosition();
        velocity = newVelocity();
    }

    public Matrix<N2, N1> xResidual(Matrix<N2, N1> a, Matrix<N2, N1> b) {
        return AngleStatistics.angleResidual(a, b, 0);
    }

    public Matrix<N2, N1> xAdd(Matrix<N2, N1> a, Matrix<N2, N1> b) {
        return AngleStatistics.angleAdd(a, b, 0);
    }

    public Sensor<N2, N1, N1> position() {
        return position;
    }

    public Sensor<N2, N1, N1> velocity() {
        return velocity;
    }

    public Sensor<N2, N1, N2> full() {
        return full;
    }

    public Matrix<N2, N1> xNormalize(Matrix<N2, N1> xmat) {
        Matrix<N2, N1> x = xmat.copy();
        x.set(0, 0, MathUtil.angleModulus(x.get(0, 0)));
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

    public abstract Sensor<N2, N1, N2> newFull();

    public abstract Sensor<N2, N1, N1> newPosition();

    public abstract Sensor<N2, N1, N1> newVelocity();
}
