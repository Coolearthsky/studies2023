package edu.unc.robotics.prrts.kdtree;

import java.util.function.BiConsumer;

/**
 * KDTraversal
 *
 * @author jeffi
 */
public interface KDTraversal<V> {
    double distToLastNearest();

    void insert(double[] config, V value);

    V nearest(double[] target);

    void near(double[] target, double radius, BiConsumer<V, Double> consumer);
}
