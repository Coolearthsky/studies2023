package org.team100.lib.reference;

import org.junit.jupiter.api.Test;
import org.team100.lib.estimator.ExtendedAngleEstimator;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.DoubleIntegrator1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ReferenceGeneratorTest {
    NonlinearPlant<N2, N1, N2> system = new DoubleIntegrator1D();

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
        ExtendedAngleEstimator<N2, N1> eae = new ExtendedAngleEstimator<N2, N1>(
                Nat.N2(),
                Nat.N1(),
                system,
                stateStdDevs,
                measurementStdDevs,
                dtSeconds);
        ReferenceGenerator rg = new ReferenceGenerator(eae);

    }

}
