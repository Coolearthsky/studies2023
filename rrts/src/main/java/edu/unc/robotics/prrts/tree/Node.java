package edu.unc.robotics.prrts.tree;

/**
 * Represents a single configuration in the RRT* tree. The path to the
 * node can be computed by following the parents until null, and then
 * reversing the order.
 */
public class Node implements Point {
    private final double[] _config;
    private final boolean _inGoal;

    /** link from the parent node */
    private Link _incoming;

    public Node(double[] config, boolean inGoal) {
        _config = config;
        _inGoal = inGoal;
        _incoming = new Link(this);
    }

    /**
     * Create a new node and
     * 
     * @param config   state of this node
     * @param inGoal   true if the state is within the goal
     * @param linkDist distance to the parent
     * @param parent   link pointing to this node (node is head of the link)
     */
    public Node(double[] config, boolean inGoal, double linkDist, Link parent) {
        _config = config;
        _inGoal = inGoal;
        // link from parent to this node
        Link link = new Link(parent.get_target(), this, linkDist);
        _incoming = link;
    }

    /**
     * Change the parent of this node.
     * 
     * @return the new parent link
     */
    public Link setLink(double linkDist, Link parent) {
        Link newLink = new Link(parent.get_target(), this, linkDist);
        _incoming = newLink;
        return newLink;
    }

    @Override
    public double[] get_config() {
        return _config;
    }

    public boolean is_inGoal() {
        return _inGoal;
    }

    public Link get_incoming() {
        return _incoming;
    }

    public Node get_parent_node() {
        return _incoming.get_source();
    }
}
