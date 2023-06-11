package org.team100.lib.reference;

import org.junit.jupiter.api.Test;
import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ReferenceGeneratorTest {

    @Test
    public void testSimple() {
        NonlinearPlant<N2, N1, N2> plant = new DoubleIntegratorRotary1D();
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(plant);
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        ReferenceGenerator rg = new ReferenceGenerator();
    }
}
