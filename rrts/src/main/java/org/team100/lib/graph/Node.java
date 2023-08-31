package org.team100.lib.graph;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import org.team100.lib.space.Point;

public class Node implements Point {
    private final double[] state;

    /** Nullable for root, can be updated. */
    private LinkInterface incoming;

    private Set<LinkInterface> outgoing;

    public Node(double[] state) {
        this.state = state;
        this.outgoing = new HashSet<LinkInterface>();
    }

    @Override
    public double[] getState() {
        return state;
    }

    public void setIncoming(LinkInterface link) {
        incoming = link;
    }

    /** Nullable for root (i.e. start). */
    public LinkInterface getIncoming() {
        return incoming;
    }

    public void addOutgoing(LinkInterface link) {
        outgoing.add(link);
    }

    public void removeOutgoing(LinkInterface link) {
        outgoing.remove(link);
    }

    public Iterator<LinkInterface> getOutgoing() {
        return outgoing.iterator();
    }

    public int getOutgoingCount() {
        return outgoing.size();
    }

    /** The path distance of the incoming link, if any. */
    public double getPathDist() {
        if (incoming == null)
            return 0;
        return incoming.get_pathDist();
    }

    @Override
    public String toString() {
        return "Node [state=" + Arrays.toString(state) + "]";
    }
}
