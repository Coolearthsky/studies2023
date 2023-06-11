package org.team100.lib.system;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** to make testing easier */
public  class MockNonlinearPlant<States extends Num, Inputs extends Num, Outputs extends Num> implements NonlinearPlant<States, Inputs, Outputs> {

    @Override
    public RandomVector<States> f(RandomVector<States> x, Matrix<Inputs, N1> u) {
        throw new UnsupportedOperationException("Unimplemented method 'f'");
    }

    @Override
    public Sensor<States, Inputs, Outputs> full() {
        throw new UnsupportedOperationException("Unimplemented method 'full'");
    }

    @Override
    public Matrix<Inputs, N1> limit(Matrix<Inputs, N1> u) {
        throw new UnsupportedOperationException("Unimplemented method 'limit'");
    }

    @Override
    public Nat<States> states() {
        throw new UnsupportedOperationException("Unimplemented method 'states'");
    }

    @Override
    public Nat<Inputs> inputs() {
        throw new UnsupportedOperationException("Unimplemented method 'inputs'");
    }

    @Override
    public Nat<Outputs> outputs() {
        throw new UnsupportedOperationException("Unimplemented method 'outputs'");
    }

    @Override
    public Matrix<Inputs, N1> finv(RandomVector<States> x, RandomVector<States> xdot) {
        throw new UnsupportedOperationException("Unimplemented method 'finv'");
    }

}