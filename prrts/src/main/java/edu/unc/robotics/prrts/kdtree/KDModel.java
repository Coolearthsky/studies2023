package edu.unc.robotics.prrts.kdtree;

import edu.unc.robotics.prrts.State;

/**
 * KDModel
 *
 * @author jeffi
 */
public interface KDModel<T extends State> {
    int dimensions();
    void getBounds(T min, T max);
    T getMin();
    T getMax();
    /** Distance Metric */
    double dist(T a, T b);
}
