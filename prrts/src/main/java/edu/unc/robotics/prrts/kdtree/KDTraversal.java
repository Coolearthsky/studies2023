package edu.unc.robotics.prrts.kdtree;

import edu.unc.robotics.prrts.State;

/**
 * KDTraversal
 *
 * @author jeffi
 */
public interface KDTraversal<T extends State, V> {
    double distToLastNearest();

    void insert(T config, V value);

    V nearest(T target);

    int near(T target, double radius, KDNearCallback<T, V> callback);
}
