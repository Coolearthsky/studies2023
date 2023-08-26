package org.team100.lib.graph;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.team100.lib.space.Path;

/**
 * This link type keeps a consistent "path distance" and must be
 * updated by the rewiring process to keep it consistent.
 * 
 * Walking the children to update the cache turns out to be faster
 * than walking the parents on every query.
 */
public class PathDistanceCachingLink implements LinkInterface {
    /** nullable for root */
    private final Node _source;
    /** nonnull */
    private final Node _target;
    /** length, i.e. cost, of this edge */
    private final double _linkDist;
    /** Total path length, i.e. cost, so far. This is updated by rewiring. */
    private double _pathDist;

    /**
     * Create a new link pointing at the node, linkDist away from parent.
     * 
     * @param source
     * @param target
     * @param linkDist distance to the parent
     */
    public PathDistanceCachingLink(Node source, Node target, double linkDist) {
        if (source == null)
            throw new IllegalArgumentException();
        if (target == null)
            throw new IllegalArgumentException();
        if (linkDist < 0)
            throw new IllegalArgumentException();

        double pathDist = source.getPathDist() + linkDist;

        _target = target;
        _linkDist = linkDist;
        _pathDist = pathDist;
        _source = source;
    }

    @Override
    public Path path() {
        Node node = get_target();
        List<double[]> configs = new LinkedList<double[]>();
        double pathDist = get_pathDist();
        while (true) {
            configs.add(node.getState());
            LinkInterface incoming = node.getIncoming();
            if (incoming == null)
                break;
            node = incoming.get_source();
        }
        Collections.reverse(configs);
        return new Path(pathDist, configs);
    }

    @Override
    public Node get_source() {
        return _source;
    }

    /** nonnull */
    @Override
    public Node get_target() {
        return _target;
    }

    @Override
    public double get_linkDist() {
        return _linkDist;
    }

    @Override
    public void set_PathDist(double d) {
        _pathDist = d;
    }

    /** Total path length from start to here */
    @Override
    public double get_pathDist() {
        return _pathDist;
    }
}
