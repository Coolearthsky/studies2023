package org.team100.lib.planner;

import org.team100.lib.graph.Node;
import org.team100.lib.space.Path;

/**
 * Interface for solvers, can be stepped and listeners can see incremental
 * results.
 */
public interface Solver {
    /** Used to adjust radius. */
    void setStepNo(int stepNo);

    /** Try to add an edge, return true if successful. */
    boolean step();

    /** Return the whole tree. */
    Iterable<Node> getNodes();

    /** The best path so far, or null if no path spans the start and end states. */
    Path getBestPath();
}
