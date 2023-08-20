package edu.unc.robotics.prrts;

import edu.unc.robotics.prrts.tree.Link;
import edu.unc.robotics.prrts.tree.Node;

import java.util.logging.Level;
import java.util.logging.Logger;

/** Utility class for path operations. */
public class Operations {
    private static final Logger _log = Logger.getLogger(Operations.class.getName());

    /**
     * If link is a shorter solution than bestPath, set bestPath to link.
     * 
     * @return best path (whichever is better)
     */
    public static Link updateBestPath(final Link bestPath, final Link link) {
        if (!link.get_node().is_inGoal()) {
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
     * Move all the children of oldParent to newParent.
     * 
     * @return new best path
     */
    public static Link updateChildren(
            final Link bestPath,
            Link newParent,
            Link oldParent) {
        if (newParent.get_node() != oldParent.get_node()) {
            _log.log(Level.WARNING, "updating links of different nodes");
            return bestPath;
        }
        if (!oldParent.isExpired()) {
            _log.log(Level.WARNING, "updating non-expired link");
            return bestPath;
        }
        if (newParent.get_pathDist() > oldParent.get_pathDist()) {
            _log.log(Level.WARNING, "attempted to update to longer path");
            return bestPath;
        }
        Link newBestPath = bestPath;
        while (true) {
            Link oldChild = oldParent.removeFirstChild();

            if (oldChild == null) {
                // done.

                if (newParent.isExpired()) {
                    oldParent = newParent;
                    newParent = oldParent.get_node().get_link();
                    continue;
                }

                return newBestPath;
            }

            if (oldChild.isExpired()) {
                continue;
            }

            Node node = oldChild.get_node();

            if (node.get_link().get_parent_node() != oldParent.get_node()) {
                continue;
            }

            Link newChild = node.setLink(oldChild.get_linkDist(), newParent);

            if (newChild != null) {
                updateChildren(newBestPath, newChild, oldChild);
                newBestPath = updateBestPath(newBestPath, newChild);
            } else {
                if (node.get_link() == oldChild) {
                    _log.log(Level.WARNING, "weird child situation");
                }
            }
        }
    }

    /**
     * Rewire oldLink to newParent
     * 
     * @return updated best path
     */
    public static Link rewire(
            final Link _bestPath,
            RobotModel _robotModel,
            Link oldLink,
            double linkDist,
            Node newParent) {
        if (oldLink.get_parent_node() == null) {
            _log.log(Level.WARNING, "attempted to rewire the root");
            return _bestPath;
        }
        if (oldLink.get_parent_node() == newParent) {
            _log.log(Level.WARNING, "attempted to rewire to current parent");
            return _bestPath;
        }

        Link newParentLink = newParent.get_link();

        double newPathDist = newParentLink.get_pathDist() + linkDist;

        // if the new path is not better, then return the old path
        if (newPathDist >= oldLink.get_pathDist()) {
            return _bestPath;
        }

        // if the new link is not feasible, return the old path
        if (!_robotModel.link(oldLink.get_node().get_config(), newParent.get_config())) {
            return _bestPath;
        }

        Link newBestPath = _bestPath;

        Link newLink = oldLink.get_node().setLink(linkDist, newParentLink);

        Operations.updateChildren(newBestPath, newLink, oldLink);
        newBestPath = Operations.updateBestPath(newBestPath, newLink);

        if (newParentLink.isExpired()) {
            Operations.updateChildren(newBestPath, newParentLink.get_node().get_link(), newParentLink);
        }

        oldLink.get_parent().removeChild(oldLink);
        return newBestPath;
    }

}
