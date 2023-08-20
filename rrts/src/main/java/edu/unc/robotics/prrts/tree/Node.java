package edu.unc.robotics.prrts.tree;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

public class Node implements Point {
    private final double[] state;

    /** Nullable for root, can be updated. */
    private Link incoming;

    private Set<Link> outgoing;

    public Node(double[] state) {
        this.state = state;
        this.outgoing = new HashSet<Link>();
    }

    @Override
    public double[] getState() {
        return state;
    }

    public void setIncoming(Link link) {
        incoming = link;
    }

    /** Nullable for root (i.e. start). */
    public Link getIncoming() {
        return incoming;
    }

    public void addOutgoing(Link link) {
        outgoing.add(link);
    }

    public void removeOutgoing(Link link) {
        outgoing.remove(link);
    }

    public Iterator<Link> getOutgoing() {
        return outgoing.iterator();
    }
}
