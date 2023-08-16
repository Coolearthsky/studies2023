package edu.unc.robotics.prrts;

/**
 * RobotModel
 *
 * @author jeffi
 */
public interface RobotModel {
    /**
     * Checks if a configuration is a goal configuration. If we are
     * searching for a path to a target goal configuration, this method
     * always returns false, since we will check the configuration can
     * extend to the target later.
     *
     * @param config the configuration to test
     * @return true if the configuration is in the goal region
     */
    boolean goal(double[] config);

    /**
     * Checks for obstacles.
     * 
     * @return true if the config is not within an obstacle
     */
    boolean clear(double[] config);

    /** Checks if a link from config a to b is feasible
     * 
     * @return true if the link is feasible
     */
    boolean link(double[] a, double[] b);
}
