package org.team100.lib.index;

import org.team100.lib.graph.Node;

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

    void setStepNo(int stepNo);
    void setRadius(double radius);

    /**
     * return a point in the same direction as the input newConfig
     * relative to nearConfig, but only dist of the way there. Note this should
     * match the metric used by dist, which might be complicated for noneuclidean
     * spaces.
     * 
     * In general this function is intended to produce a feasible trajectory
     * starting at nearconfig; ideally ending at newConfig (double boundary problem)
     * but any trajectory will do.
     * 
     * @param stepNo adjusts steering, choosing progressively closer nodes
     * @param nearConfig the nearest config
     * @param newConfig  the candidate config
     * @param dist       the fraction to go
     * @return steered new config
     */
    double[] steer(KDNearNode<Node> x_nearest,  double[] newConfig);

}
