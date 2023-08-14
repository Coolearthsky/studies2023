package edu.unc.robotics.prrts;

/**
 * RobotModel
 *
 * @author jeffi
 */
public interface RobotModel<T extends State> {
    boolean goal(T config);

    boolean clear(T config);

    boolean link(T a, T b);
}
