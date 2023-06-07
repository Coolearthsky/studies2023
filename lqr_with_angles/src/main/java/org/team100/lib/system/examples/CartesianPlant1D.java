package org.team100.lib.system.examples;

import org.team100.lib.math.RandomVector;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.Sensor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Base class for one-dimensional cartesian plants. */
public abstract class CartesianPlant1D implements NonlinearPlant<N2, N1, N2>  {
    private final Sensor<N2, N1, N2> full;
    private final Sensor<N2, N1, N1> position;
    private final Sensor<N2, N1, N1> velocity;

    public abstract class FullSensor implements Sensor<N2, N1, N2> {
        public RandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
            return x;
        }

        public RandomVector<N2> yResidual(RandomVector<N2> a, RandomVector<N2> b) {
            return a.minus(b);
        }

        public Nat<N2> rows() {
            return Nat.N2();
        }
    }

    public abstract class PositionSensor implements Sensor<N2, N1, N1> {
        public RandomVector<N1> h(RandomVector<N2> x, Matrix<N1, N1> u) {
            // return VecBuilder.fill(x.get(0, 0));
            return new RandomVector<>(x.x.block(Nat.N1(),Nat.N1(), 0, 0), x.P.block(Nat.N1(), Nat.N1(), 0, 0));
        }

        public RandomVector<N1> yResidual(RandomVector<N1> a, RandomVector<N1> b) {
            return a.minus(b);
        }

        public Nat<N1> rows() {
            return Nat.N1();
        }
    }

    public abstract class VelocitySensor implements Sensor<N2, N1, N1> {
        public RandomVector<N1> h(RandomVector<N2> x, Matrix<N1, N1> u) {
            // return VecBuilder.fill(x.get(1, 0));
            return new RandomVector<>(x.x.block(Nat.N1(),Nat.N1(), 1, 0), x.P.block(Nat.N1(), Nat.N1(), 1, 1));

        }

        public RandomVector<N1> yResidual(RandomVector<N1> a, RandomVector<N1> b) {
            return a.minus(b);
        }

        public Nat<N1> rows() {
            return Nat.N1();
        }
    }


    public CartesianPlant1D() {
        full = newFull();
        position = newPosition();
        velocity = newVelocity();
    }

    /**
     * State dimension 0 is an angle.
     */
    @Override
    public RandomVector<N2> xResidual(RandomVector<N2> a, RandomVector<N2> b) {
        return a.minus(b);
    }

    @Override
    public RandomVector<N2> xAdd(RandomVector<N2> a, RandomVector<N2> b) {
        return a.plus(b);
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
    
    public RandomVector<N2> xNormalize(RandomVector<N2> xmat) {
        return xmat;
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
