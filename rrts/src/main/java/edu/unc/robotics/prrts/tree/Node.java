package edu.unc.robotics.prrts.tree;

/**
 * Represents a single configuration in the RRT* tree. The path to the
 * node can be computed by following the parents until null, and then
 * reversing the order.
 */
public class Node implements Point {
    private final double[] _config;

    /** link from the parent node */
    private Link _incoming;

    public Node(double[] config) {
        _config = config;
        _incoming = new Link(null, this, 0, 0);
    }

    /**
     * Create a new node
     * 
     * @param config   state of this node
     * @param linkDist distance to the parent
     * @param parent   parent Node
     */
    public Node(double[] config, double linkDist, Node parent) {
        _config = config;
        _incoming = new Link(parent, this, linkDist);
    }

    /**
     * Change the parent of this node.
     * 
     * @return the new parent link
     */
    public Link setLink(double linkDist, Node parent) {
        _incoming = new Link(parent, this, linkDist);
        return _incoming;
    }

    @Override
    public double[] get_config() {
        return _config;
    }

    public Link get_incoming() {
        return _incoming;
    }
}
