package edu.unc.robotics.prrts.kdtree;

/**
 * KDModel
 *
 * @author jeffi
 */
public interface KDModel {
    int dimensions();

    double[] getMin();

    double[] getMax();

    /**
     * Distance, i.e. cost, between states. Note that this function is not, in
     * general, symmetric.
     * 
     * @return cost
     */
    double dist(double[] start, double[] end);

    /**
     * adjust newConfig so that it's in the same direction as the input newConfig
     * relative to nearConfig, but only dist of the way there. Note this should
     * match the metric used by dist, which might be complicated for noneuclidean
     * spaces.
     * 
     * @param nearConfig the nearest config
     * @param newConfig  INOUT the candidate config
     * @param dist       the fraction to go
     */
    void steer(double[] nearConfig, double[] newConfig, double dist);

}
