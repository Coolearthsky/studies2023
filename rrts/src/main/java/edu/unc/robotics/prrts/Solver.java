package edu.unc.robotics.prrts;

import edu.unc.robotics.prrts.tree.Node;

/**
 * Interface for solvers, can be stepped and listeners can see incremental
 * results.
 */
public interface Solver {
    /** Try to add an edge, return true if successful. */
    boolean step(int stepNo);

    /** Return the whole tree. */
    Iterable<Node> getNodes();

    /** The best path so far, or null if no path spans the start and end states. */
    Path getBestPath();
}
