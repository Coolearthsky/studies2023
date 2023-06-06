package org.team100.lib.math;

import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalJacobian;

/**
 * Wraps the WPI version but for random variables.
 * 
 * Ignores the covariances, just passes the means to the WPI class.
 */
public class Jacobian {
    // for the first method
    static class acceptableFunction<Cols extends Num, States extends Num>
            implements Function<Matrix<Cols, N1>, Matrix<States, N1>> {
        private final Nat<Cols> cols;
        private final Function<RandomVector<Cols>, RandomVector<States>> f;

        public acceptableFunction(
                Nat<Cols> cols,
                Function<RandomVector<Cols>, RandomVector<States>> f) {
            this.cols = cols;
            this.f = f;
        }

        @Override
        public Matrix<States, N1> apply(Matrix<Cols, N1> arg0) {
            RandomVector<Cols> arg = new RandomVector<>(arg0, new Matrix<>(cols, cols));
            RandomVector<States> result = f.apply(arg);
            return result.x;
        }
    }

    // for the second method
    static class acceptableBiFunction<States extends Num, Inputs extends Num, Outputs extends Num>
            implements BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<Outputs, N1>> {
        private final Nat<States> states;
        private final BiFunction<RandomVector<States>, Matrix<Inputs, N1>, Matrix<Outputs, N1>> f;

        public acceptableBiFunction(
                Nat<States> states,
                BiFunction<RandomVector<States>, Matrix<Inputs, N1>, Matrix<Outputs, N1>> f) {
            this.states = states;
            this.f = f;
        }

        @Override
        public Matrix<Outputs, N1> apply(Matrix<States, N1> arg0, Matrix<Inputs, N1> arg1) {
            RandomVector<States> a0 = new RandomVector<>(arg0, new Matrix<>(states, states));
            return f.apply(a0, arg1);
        }
    }

    // for the third method
    static class acceptableBiFunction2<States extends Num, Inputs extends Num>
            implements BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> {
        private final Nat<States> states;
        private final BiFunction<RandomVector<States>, Matrix<Inputs, N1>, RandomVector<States>> f;

        public acceptableBiFunction2(
                Nat<States> states,
                BiFunction<RandomVector<States>, Matrix<Inputs, N1>, RandomVector<States>> f) {
            this.states = states;
            this.f = f;
        }

        @Override
        public Matrix<States, N1> apply(Matrix<States, N1> arg0, Matrix<Inputs, N1> arg1) {
            RandomVector<States> a0 = new RandomVector<>(arg0, new Matrix<>(states, states));
            return f.apply(a0, arg1).x;
        }
    }

    public static <Rows extends Num, Cols extends Num, States extends Num> Matrix<Rows, Cols> numericalJacobian(
            Nat<Rows> rows,
            Nat<Cols> cols,
            Function<RandomVector<Cols>, RandomVector<States>> f,
            RandomVector<Cols> x) {
        return NumericalJacobian.numericalJacobian(rows, cols, new acceptableFunction<>(cols, f), x.x);
    }

    public static <Rows extends Num, States extends Num, Inputs extends Num, Outputs extends Num> Matrix<Rows, States> numericalJacobianX(
            Nat<Rows> rows,
            Nat<States> states,
            BiFunction<RandomVector<States>, Matrix<Inputs, N1>, Matrix<Outputs, N1>> f,
            Matrix<States, N1> x,
            Matrix<Inputs, N1> u) {
        return NumericalJacobian.numericalJacobianX(rows, states, new acceptableBiFunction<>(states, f), x, u);
    }

    public static <Rows extends Num, States extends Num, Inputs extends Num> Matrix<Rows, Inputs> numericalJacobianU(
            Nat<Rows> rows,
            Nat<Inputs> inputs,
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            Matrix<States, N1> x,
            Matrix<Inputs, N1> u) {
        return NumericalJacobian.numericalJacobianU(rows, inputs, new acceptableBiFunction2<>(states, f), x, u);
    }

}
