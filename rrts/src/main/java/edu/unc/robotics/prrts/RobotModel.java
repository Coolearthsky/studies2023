package edu.unc.robotics.prrts;

/**
 * RobotModel
 *
 * @author jeffi
 */
public interface RobotModel {

    /** Initial state. */
    double[] initial();

    /**
     * @return true if the configuration is in the goal region
     */
    boolean goal(double[] config);

    /**
     * Checks for obstacles.
     * 
     * @return true if the config is not within an obstacle
     */
    boolean clear(double[] config);

    /**
     * @return true if the link is feasible
     */
    boolean link(double[] source, double[] target);
}
