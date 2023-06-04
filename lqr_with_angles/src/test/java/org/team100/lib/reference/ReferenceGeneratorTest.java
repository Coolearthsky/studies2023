package org.team100.lib.reference;

import org.junit.jupiter.api.Test;
import org.team100.lib.estimator.NonlinearEstimator;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.NormalDoubleIntegratorRotary1D;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ReferenceGeneratorTest {

    @Test
    public void testSimple() {
        NonlinearPlant<N2, N1, N2> plant = new NormalDoubleIntegratorRotary1D();
        double dtSeconds = 0.02;
        NonlinearEstimator<N2, N1, N2> eae = new NonlinearEstimator<>(plant, dtSeconds);
        ReferenceGenerator rg = new ReferenceGenerator(eae);
    }
}
