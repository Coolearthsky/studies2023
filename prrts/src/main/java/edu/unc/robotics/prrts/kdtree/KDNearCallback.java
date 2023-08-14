package edu.unc.robotics.prrts.kdtree;

import edu.unc.robotics.prrts.State;

/**
 * KDNearCallback
 *
 * @author jeffi
 */
public interface KDNearCallback<T extends State, V> {
    void kdNear(T target, int index, T config, V value, double dist);
}
