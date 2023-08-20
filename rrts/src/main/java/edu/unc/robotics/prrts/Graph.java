package edu.unc.robotics.prrts;

import edu.unc.robotics.prrts.tree.Link;
import edu.unc.robotics.prrts.tree.Node;

public class Graph {
    /**
     * Create a link from source to target, add it to the source outgoing set, and
     * set it as the target incoming link. Also remove the target's previous
     * incoming link from its source outgoing set, if needed.
     */
    public static Link newLink(Node source, Node target, double dist) {
        if (source == null)
            throw new IllegalArgumentException("source may not be null");
        if (target == null)
            throw new IllegalArgumentException("target may not be null");
        if (dist < 0)
            throw new IllegalArgumentException("dist may not be negative");

        Link oldLink = target.getIncoming();
        if (oldLink != null)
            oldLink.get_source().removeOutgoing(oldLink);

        Link link = new Link(source, target, dist);
        target.setIncoming(link);
        source.addOutgoing(link);
        return link;
    }
}
