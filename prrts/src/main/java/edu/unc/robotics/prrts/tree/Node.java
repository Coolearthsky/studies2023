package edu.unc.robotics.prrts.tree;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Just shrinking the enormous PRRTStar class
 * 
 * Represents a single configuration in the RRT* tree. The path to the
 * node can be computed by following the parents until null, and then
 * reversing the order. This class is part of the public API, but is
 * also used internally. The package-private members are intentionally
 * not part of the public API as they are subject to change.
 *
 * The public API may safely be accessed while the PRRTStar is running.
 * There is a possibility that the path to a node will change while it
 * is being accessed, but the config member will not change. For
 * efficiency, the config member is exposed as a direct reference an array.
 * It should NOT be modified by the caller.
 */
public class Node {
    private final double[] _config;
    private final boolean _inGoal;
    private final AtomicReference<Link> _link;

    public Node(double[] config, boolean inGoal) {
        _config = config;
        _inGoal = inGoal;
        _link = new AtomicReference<>(new Link(this));
    }

    public Node(double[] config, boolean inGoal, double linkDist, Link parent) {
        _config = config;
        _inGoal = inGoal;
        Link link = new Link(this, linkDist, parent);
        _link = new AtomicReference<>(link);
        parent.addChild(link);
    }

    public Link setLink(Link oldLink, double linkDist, Link parent) {
        Link newLink = new Link(this, linkDist, parent);
        if (!_link.compareAndSet(oldLink, newLink)) {
            return null;
        }
        assert newLink.get_pathDist() <= oldLink.get_pathDist();
        parent.addChild(newLink);
        return newLink;
    }

    public double[] get_config() {
        return _config;
    }

    public boolean is_inGoal() {
        return _inGoal;
    }

    public AtomicReference<Link> get_link() {
        return _link;
    }
}
