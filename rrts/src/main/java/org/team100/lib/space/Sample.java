package org.team100.lib.space;

import java.util.Random;

import org.team100.lib.index.KDModel;
import org.team100.lib.random.MersenneTwister;

/**
 * Supplies samples drawn from a uniform distribution across the model bounds.
 */
public class Sample {
    private final KDModel _kdModel;
    private final double[] _sampleMin;
    private final double[] _sampleMax;
    private final Random _random;

    public Sample(KDModel kdModel) {
        _kdModel = kdModel;
        _random = new MersenneTwister(0);
        _sampleMin = _kdModel.getMin();
        _sampleMax = _kdModel.getMax();
    }

    public double[] get() {
        double[] result = new double[_kdModel.dimensions()];
        for (int i = _kdModel.dimensions(); --i >= 0;) {
            double range = _sampleMax[i] - _sampleMin[i];
            result[i] = _sampleMin[i] + range * _random.nextDouble();
        }
        return result;
    }
}
