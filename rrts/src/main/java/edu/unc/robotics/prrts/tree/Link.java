package edu.unc.robotics.prrts.tree;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import edu.unc.robotics.prrts.Path;

public class Link {
    /** nullable for root */
    private final Node _source;
    /** nonnull */
    private final Node _target;
    /** length, i.e. cost, of this edge */
    private final double _linkDist;
    /** Total path length, i.e. cost, so far.  This is updated by rewiring. */
    private double _pathDist;

    /**
     * Create a new link pointing at the node, linkDist away from parent.
     * 
     * @param source
     * @param target
     * @param linkDist distance to the parent
     */
    public Link(Node source, Node target, double linkDist) {
        if (source == null) throw new IllegalArgumentException();
        Link incoming = source.getIncoming();
        double pathDist;
        if (incoming == null) { // parent is root
            pathDist = linkDist;
        } else {
            pathDist = incoming.get_pathDist() + linkDist;
        }
        if (target == null)
            throw new IllegalArgumentException();
        if (linkDist < 0)
            throw new IllegalArgumentException();
        if (pathDist < 0)
            throw new IllegalArgumentException();
        _target = target;
        _linkDist = linkDist;
        _pathDist = pathDist;
        _source = source;

    }

    public Path path() {
        Node node = get_target();
        List<double[]> configs = new LinkedList<double[]>();
        double pathDist = get_pathDist();
        while (true) {
            configs.add(node.getState());
            Link incoming = node.getIncoming();
            if (incoming == null) break;
            node = incoming.get_source();
        }
        Collections.reverse(configs);
        return new Path(pathDist, configs);
    }

    public Node get_source() {
        return _source;
    }

    /** nonnull */
    public Node get_target() {
        return _target;
    }

    public double get_linkDist() {
        return _linkDist;
    }

    public void set_PathDist(double d) {
        _pathDist = d;
    }

    /** Total path length from start to here */
    public double get_pathDist() {
        return _pathDist;
    }
}
