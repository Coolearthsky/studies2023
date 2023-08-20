package org.team100.lib.graph;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import org.team100.lib.space.Point;

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

    /** The path distance of the incoming link, if any. */
    public double getPathDist() {
        if (incoming == null)
            return 0;
        return incoming.get_pathDist();
    }
}
