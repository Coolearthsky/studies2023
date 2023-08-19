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
     */
    public static void updateChildren(
            final Link bestPath,
            Link newParent,
            Link oldParent) {
        if (newParent.get_node() != oldParent.get_node()) {
            _log.log(Level.WARNING, "updating links of different nodes");
            return;
        }
        if (!oldParent.isExpired()) {
            _log.log(Level.WARNING, "updating non-expired link");
            return;
        }
        if (newParent.get_pathDist() > oldParent.get_pathDist()) {
            _log.log(Level.WARNING, "attempted to update to longer path");
            return;
        }
        for (;;) {
            Link oldChild = oldParent.removeFirstChild();

            if (oldChild == null) {
                // done.

                if (newParent.isExpired()) {
                    oldParent = newParent;
                    newParent = oldParent.get_node().get_link();
                    continue;
                }

                return;
            }

            if (oldChild.isExpired()) {
                continue;
            }

            Node node = oldChild.get_node();

            // if (node.get_link().get().get_parent().get_node() != oldParent.get_node()) {
            if (node.get_link().get_parent_node() != oldParent.get_node()) {
                continue;
            }

            Link newChild = node.setLink(oldChild, oldChild.get_linkDist(), newParent);

            if (newChild != null) {
                updateChildren(bestPath, newChild, oldChild);
                updateBestPath(bestPath, newChild);
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

        Node node = oldLink.get_node();

        Link parentLink = newParent.get_link();

        double pathDist = parentLink.get_pathDist() + linkDist;

        // check if rewiring would create a shorter path
        if (pathDist >= oldLink.get_pathDist()) {
            return _bestPath;
        }

        // check if rewiring is possible
        if (!_robotModel.link(oldLink.get_node().get_config(), newParent.get_config())) {
            return _bestPath;
        }

        // rewire the node. this loop continues to attempt atomic
        // updates until either the update succeeds or the pathDist
        // of the oldLink is found to be better than what we're trying
        // to put in
        Link newBestPath = _bestPath;
        do {

            Link newLink = node.setLink(oldLink, linkDist, parentLink);

            if (newLink != null) {
                Operations.updateChildren(newBestPath, newLink, oldLink);
                newBestPath = Operations.updateBestPath(newBestPath, newLink);

                if (parentLink.isExpired()) {
                    Operations.updateChildren(newBestPath, parentLink.get_node().get_link(), parentLink);
                }

                // Setting newLink expires oldLink but doesn not remove
                // it from its parent. Here we do a little cleanup.
                // We do it after the expired parent check since the parent
                // will likely have already cleaned this up, and this call
                // will be O(1) instead of O(n)
                oldLink.get_parent().removeChild(oldLink);
                return newBestPath;
            }

            Link updatedOldLink = node.get_link();

            if (updatedOldLink == oldLink) {
                _log.log(Level.WARNING, "update isn't different");
            }

            oldLink = updatedOldLink;

        } while (pathDist < oldLink.get_pathDist());
        
        return newBestPath;
    }

}
