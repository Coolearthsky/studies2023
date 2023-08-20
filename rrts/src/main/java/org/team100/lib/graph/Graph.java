package org.team100.lib.graph;

import java.util.Iterator;

import edu.unc.robotics.prrts.RobotModel;

public class Graph {
    /**
     * Create a link from source to target, add it to the source outgoing set, and
     * set it as the target incoming link. Also remove the target's previous
     * incoming link from its source outgoing set, if needed.
     * 
     * @return the newly created link, never null.
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

	/**
	 * Walks the incoming path to calculate the total path distance to the specified
	 * node.
	 */
	public static double getPathDist(Node node) {
	    double pathDist = 0;
	    while (true) {
	        Link incoming = node.getIncoming();
	        if (incoming == null)
	            return pathDist;
	        pathDist += incoming.get_linkDist();
	        node = incoming.get_source();
	    }
	}

    /** Walks the outgoing subtree and updates the path lengths of each link. */
    public static void updatePathLengths(Link link) {
        Node node = link.get_target();
        Iterator<Link> iter = node.getOutgoing();
        while (iter.hasNext()) {
            Link child = iter.next();
            child.set_PathDist(child.get_linkDist() + link.get_pathDist());
            updatePathLengths(child);
        }
    }

    /**
     * @return best path (link or bestpath, whichever is shorter)
     */
    public static Link chooseBestPath(RobotModel model, final Link oldLink, final Link newLink) {
        if (newLink == null)
            throw new IllegalArgumentException();
            
        if (!model.goal(newLink.get_target().getState()))
            return oldLink;
    
        if (oldLink == null)
            return newLink;
    
        if (newLink.get_pathDist() < oldLink.get_pathDist())
            return newLink;
    
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
        Link oldLink = target.getIncoming();
        oldLink.get_source().removeOutgoing(oldLink);
        Link newLink = newLink(source, target, linkDist);
    
        // Update all the child path lengths for consistency.
        updatePathLengths(newLink);
    
        return true;
    }
}
