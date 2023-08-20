package edu.unc.robotics.prrts;

import edu.unc.robotics.prrts.tree.Link;
import edu.unc.robotics.prrts.tree.Node;

import java.util.Iterator;
import java.util.logging.Level;
import java.util.logging.Logger;

/** Utility class for path operations. */
public class Operations {
    private static final Logger _log = Logger.getLogger(Operations.class.getName());

    /**
     * @return best path (link or bestpath, whichever is shorter)
     */
    public static Link updateBestPath(RobotModel model, final Link bestPath, final Link link) {
        if (link == null)
            throw new IllegalArgumentException();
        boolean isInGoal = model.goal(link.get_target().getState());
        if (!isInGoal) {
            return bestPath;
        }

        if (bestPath != null) {
            double bestDist = bestPath.get_pathDist();
            if (link.get_pathDist() >= bestDist) {
                return bestPath;
            }
        }
        return link;
    }

    /**
     * Rewire the target node to newParent
     * 
     * @return updated best path
     */
    public static Link rewire(
            final Link _bestPath,
            RobotModel _robotModel,
            Node target,
            double linkDist,
            Node newParent) {
        if (target.getIncoming() == null) {
            _log.log(Level.WARNING, "attempted to rewire the root");
            return _bestPath;
        }
        if (target.getIncoming().get_source() == newParent) {
            _log.log(Level.WARNING, "attempted to rewire to current parent");
            return _bestPath;
        }

        Link newParentLink = newParent.getIncoming();

        double newPathDist;
        if (newParentLink == null) { // newparent is root
            newPathDist = linkDist;
        } else {
            newPathDist = newParentLink.get_pathDist() + linkDist;
        }

        // if the new path is not better, then return the old path
        if (newPathDist >= target.getIncoming().get_pathDist()) {
            return _bestPath;
        }

        // if the new link is not feasible, return the old path
        if (!_robotModel.link(target.getState(), newParent.getState())) {
            return _bestPath;
        }

        // actually make and set the new link
        Link oldLink = target.getIncoming();
        oldLink.get_source().removeOutgoing(oldLink);
        Link newLink = Graph.newLink(newParent, target, linkDist);

        // update all the child path lengths.
        updatePathLengths(newLink);

        // if this makes a new best path, return it, otherwise return the old one
        return Operations.updateBestPath(_robotModel, _bestPath, newLink);
    }

    public static void updatePathLengths(Link link) {
        Node node = link.get_target();
        Iterator<Link> iter = node.getOutgoing();
        while (iter.hasNext()) {
            Link child = iter.next();
            child.set_PathDist(child.get_linkDist() + link.get_pathDist());
            updatePathLengths(child);
        }
    }

    public static double dist(Node node) {
        double pathDistAgain = 0;
        while (true) {
            Link incoming = node.getIncoming();
            if (incoming == null)
                return pathDistAgain;
            pathDistAgain += incoming.get_linkDist();
            node = incoming.get_source();
        }
    }
}
