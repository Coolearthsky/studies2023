package edu.unc.robotics.prrts;

import java.util.Random;

import edu.unc.robotics.prrts.kdtree.KDModel;
import edu.unc.robotics.prrts.util.MersenneTwister;

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
        _random = new MersenneTwister();
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
