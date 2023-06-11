package org.team100.lib.system.examples;

import org.team100.lib.math.RandomVector;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Base class for one-dimensional rotational plants. */
public abstract class RotaryPlant1D implements NonlinearPlant<N2, N1, N2> {
    private static final double kBig = 1e9;

    @Override
    public RandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
        return x;
    }

    @Override
    public RandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
        return y;
    }

    /** default is no noise */
    @Override
    public WhiteNoiseVector<N2> w() {
        return new WhiteNoiseVector<>(new Matrix<>(Nat.N2(), Nat.N2()));
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
