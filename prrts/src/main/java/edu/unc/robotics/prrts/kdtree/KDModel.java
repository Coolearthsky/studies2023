package edu.unc.robotics.prrts.kdtree;

/**
 * KDModel
 *
 * @author jeffi
 */
public interface KDModel {
    int dimensions();

    /**
     * Write model min and max
     * 
     * @param min OUTVAR model minimum
     * @param max OUTVAR model maximum
     */
    void getBounds(double[] min, double[] max);

    double dist(double[] config, double[] target);
}
