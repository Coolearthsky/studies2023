package org.team100.lib.graph;

import java.util.Iterator;

import org.team100.lib.index.KDModel;
import org.team100.lib.planner.RobotModel;

public class Graph {
    /** use the caching link type? */
    public static boolean linkTypeCaching = true;

    /**
     * Create a link from source to target, using the model distance from the source
     * to target.
     */
    public static LinkInterface newLink(KDModel model, Node source, Node target) {
        double dist = model.dist(source.getState(), target.getState());
        // System.out.printf("newLink from [%5.3f %5.3f] to [%5.3f %5.3f] d %5.3f\n",
        //         source.getState()[0], source.getState()[1], target.getState()[0], target.getState()[1], dist);
        return newLink(source, target, dist);
    }

    /**
     * Create a link from source to target, add it to the source outgoing set, and
     * set it as the target incoming link. Also remove the target's previous
     * incoming link from its source outgoing set, if needed.
     * 
     * @return the newly created link, never null.
     */
    public static LinkInterface newLink(Node source, Node target, double dist) {
        if (source == null)
            throw new IllegalArgumentException("source may not be null");
        if (target == null)
            throw new IllegalArgumentException("target may not be null");
        if (dist < 0)
            throw new IllegalArgumentException("dist may not be negative");

        LinkInterface oldLink = target.getIncoming();
        if (oldLink != null)
            oldLink.get_source().removeOutgoing(oldLink);

        LinkInterface link;
        if (linkTypeCaching) {
            link = new PathDistanceCachingLink(source, target, dist);
        } else {
            link = new LocalLink(source, target, dist);
        }
        target.setIncoming(link);
        source.addOutgoing(link);
        return link;
    }

    /**
     * Walks the incoming path to calculate the total path distance to the specified
     * node.
     */
    public static double getPathDist(Node node) {
        double pathDist = 0;
        while (true) {
            LinkInterface incoming = node.getIncoming();
            if (incoming == null)
                return pathDist;
            pathDist += incoming.get_linkDist();
            node = incoming.get_source();
        }
    }

    /** Walks the outgoing subtree and updates the path lengths of each link. */
    public static void updatePathLengths(LinkInterface link) {
        Node node = link.get_target();
        Iterator<LinkInterface> iter = node.getOutgoing();
        while (iter.hasNext()) {
            LinkInterface child = iter.next();
            child.set_PathDist(child.get_linkDist() + link.get_pathDist());
            updatePathLengths(child);
        }
    }

    /**
     * @return best path (link or bestpath, whichever is shorter)
     */
    public static LinkInterface chooseBestPath(RobotModel model, final LinkInterface oldLink,
            final LinkInterface newLink) {
        System.out.println("try new best path");
        if (newLink == null)
            throw new IllegalArgumentException();

        if (!model.goal(newLink.get_target().getState()))
            return oldLink;

        if (oldLink == null)
            return newLink;

        if (newLink.get_pathDist() < oldLink.get_pathDist()) {
            System.out.printf("new best path %5.3f\n", newLink.get_pathDist());
            return newLink;
        }

        return oldLink;
    }

    /**
     * Rewires the target node to source.
     * 
     * @return true if it actually changed anything
     */
    public static boolean rewire(RobotModel _robotModel, Node source, Node target, double linkDist) {
        if (target.getIncoming() == null)
            throw new IllegalArgumentException("cannot rewire the root");

        if (target.getIncoming().get_source() == source)
            throw new IllegalArgumentException("vacuous rewiring");

        double newPathDist = source.getPathDist() + linkDist;

        // if the new path is not better, then return the old path
        if (newPathDist >= target.getIncoming().get_pathDist()) {
            return false;
        }

        // if the new link is not feasible, return the old path
        if (!_robotModel.link(target.getState(), source.getState())) {
            return false;
        }

        // actually make and set the new link
        LinkInterface oldLink = target.getIncoming();
        oldLink.get_source().removeOutgoing(oldLink);
        LinkInterface newLink = newLink(source, target, linkDist);

        // Update all the child path lengths for consistency.
        // but only for the link types that need it.
        if (linkTypeCaching)
            updatePathLengths(newLink);

        return true;
    }
}
