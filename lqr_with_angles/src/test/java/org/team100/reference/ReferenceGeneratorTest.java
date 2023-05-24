package org.team100.reference;

import org.junit.jupiter.api.Test;
import org.team100.estimator.ExtendedAngleEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ReferenceGeneratorTest {
    @Test
    public void testSimple() {
        Matrix<N2, N1> stateStdDevs = VecBuilder.fill(1,1);
        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(1,1);
        double dtSeconds = 0.02;
        ExtendedAngleEstimator eae = new ExtendedAngleEstimator(stateStdDevs, measurementStdDevs, dtSeconds);
        ReferenceGenerator rg = new ReferenceGenerator(eae);
        
    }
    
}
