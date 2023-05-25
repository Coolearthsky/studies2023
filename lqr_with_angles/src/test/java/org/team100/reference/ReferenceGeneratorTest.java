package org.team100.reference;

import org.junit.jupiter.api.Test;
import org.team100.estimator.ExtendedAngleEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ReferenceGeneratorTest {
    /**
     * The derivative of state.
     * 
     * x = (position, velocity)
     * xdot = (velocity, control)
     */
    Matrix<N2, N1> f(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return VecBuilder.fill(x.get(1, 0), u.get(0, 0));
    }

    /**
     * Both measurements: (position, velocity)
     */
    Matrix<N2, N1> h(Matrix<N2, N1> x, Matrix<N1, N1> u) {
        return x;
    }

    @Test
    public void testSimple() {
        Matrix<N2, N1> stateStdDevs = VecBuilder.fill(1, 1);
        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(1, 1);
        double dtSeconds = 0.02;
        ExtendedAngleEstimator eae = new ExtendedAngleEstimator(
                this::f,
                this::h,
                stateStdDevs,
                measurementStdDevs,
                dtSeconds);
        ReferenceGenerator rg = new ReferenceGenerator(eae);

    }

}
