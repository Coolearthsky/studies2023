package edu.unc.robotics.prrts.tree;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import edu.unc.robotics.prrts.Path;

/**
 * A doubly-connected directed graph which also has a linked list of siblings.
 */
public class Link {
    // nullable for root
    private final Node _source;
    // nonnull
    private final Node _target;
    /** length, i.e. cost, of this edge */
    private final double _linkDist;
    /** total path length, i.e. cost, so far */
    private final double _pathDist;

    public Link(Node root) {
        this(null, root, 0, 0);
    }

    /**
     * Create a new link pointing at the node, linkDist away from parent.
     * 
     * @param source
     * @param target
     * @param linkDist distance to the parent
     */
    public Link(Node source, Node target, double linkDist) {
        this(source, target, linkDist, source.get_incoming().get_pathDist() + linkDist);
    }

    public boolean isExpired() {
        return _target.get_incoming() != this;
    }

    public Path path() {
        Node node = get_target();
        List<double[]> configs = new LinkedList<double[]>();
        double pathDist = get_pathDist();
        while (node != null) {
            configs.add(node.get_config());
            node = node.get_parent_node();
        }
        Collections.reverse(configs);
        return new Path(pathDist, configs);
    }

    public Node get_source() {
        return _source;
    }

    public Node get_target() {
        return _target;
    }

    public double get_linkDist() {
        return _linkDist;
    }

    /** Total path length from start to here */
    public double get_pathDist() {
        return _pathDist;
    }

    //////////////////////////////////////////////////////////////////

    private Link(Node source,
            Node target,
            double linkDist,
            double pathDist) {
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

}
