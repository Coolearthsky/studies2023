package org.team100.lib.system.examples;

import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.RandomVector;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Base class for one-dimensional rotational plants. */
public abstract class RotaryPlant1D implements NonlinearPlant<N2, N1, N2> {
    private static final double kBig = 1e9;
    private final Sensor<N2, N1, N2> full;
    private final Sensor<N2, N1, N2> position;
    private final Sensor<N2, N1, N2> velocity;

    public class FullSensor implements Sensor<N2, N1, N2> {
        @Override
        public RandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
            return x;
        }

        @Override
        public RandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
            return y;
        }
    }

    public class PositionSensor implements Sensor<N2, N1, N2> {
        @Override
        public AngularRandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
            Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
            yx.set(0, 0, x.x.get(0, 0)); // pass position
            Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
            yP.set(0, 0, x.P.get(0, 0)); // variance is like state :( TODO: this variance is wrong
            yP.set(1, 1, 1e9); // velocity gets "don't know" variance
            return new AngularRandomVector<>(yx, yP);
        }

        // x0 = y0
        // x1 = 0 with high variance
        @Override
        public AngularRandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
            Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
            xx.set(0, 0, y.x.get(0, 0));
            Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
            xP.set(0, 0, y.P.get(0, 0));
            xP.set(1, 1, kBig); // which means it could be anything
            return new AngularRandomVector<>(xx, xP);
        }
    }

    public class VelocitySensor implements Sensor<N2, N1, N2> {
        @Override
        public AngularRandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
            Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
            yx.set(1, 0, x.x.get(1, 0)); // pass velocity
            Matrix<N2, N2> yP = new Matrix<>(Nat.N2(), Nat.N2());
            yP.set(0, 0, 1e9); // "don't know" variance
            yP.set(1, 1, x.P.get(1, 1)); // variance is like state :( TODO: this variance is wrong
            return new AngularRandomVector<>(yx, yP);
        }

        // x0 = 0 with high variance
        // x1 = y0
        @Override
        public AngularRandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
            Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
            xx.set(1, 0, y.x.get(1, 0)); // velocity
            Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
            xP.set(0, 0, kBig); // which means it could be anything
            xP.set(1, 1, y.P.get(1, 1));
            return new AngularRandomVector<>(xx, xP);
        }
    }

    public RotaryPlant1D() {
        full = new FullSensor();
        position = new PositionSensor();
        velocity = new VelocitySensor();
    }

    public Sensor<N2, N1, N2> position() {
        return position;
    }

    public Sensor<N2, N1, N2> velocity() {
        return velocity;
    }

    @Override
    public Sensor<N2, N1, N2> full() {
        return full;
    }

    @Override
    public Matrix<N1, N1> limit(Matrix<N1, N1> u) {
        return StateSpaceUtil.desaturateInputVector(u, 12.0);
    }

    @Override
    public Nat<N2> states() {
        return Nat.N2();
    }

    @Override
    public Nat<N1> inputs() {
        return Nat.N1();
    }

    @Override
    public Nat<N2> outputs() {
        return Nat.N2();
    }
}
