package edu.unc.robotics.prrts.tree;

/**
 * A point in some metric space, not necessarily Euclidean; the KD Tree assumes
 * Euclidean metric, though, so, like, watch out.
 */
public interface Point {
    /** The vector that describes this point. */
    double[] get_config();
}
