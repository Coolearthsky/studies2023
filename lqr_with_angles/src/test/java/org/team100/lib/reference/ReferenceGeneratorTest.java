package org.team100.lib.reference;

import org.junit.jupiter.api.Test;
import org.team100.lib.estimator.NonlinearEstimator;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.DoubleIntegrator1D;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ReferenceGeneratorTest {

    @Test
    public void testSimple() {
        NonlinearPlant<N2, N1, N2> system = new DoubleIntegrator1D(1, 1, 1, 1);
        double dtSeconds = 0.02;
        NonlinearEstimator<N2, N1,N2> eae = new NonlinearEstimator<N2, N1,N2>(
                Nat.N2(),
                Nat.N1(),
                Nat.N2(),
                system,
                dtSeconds);
        ReferenceGenerator rg = new ReferenceGenerator(eae);
    }
}
