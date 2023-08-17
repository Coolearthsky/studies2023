package edu.unc.robotics.prrts;

import java.util.concurrent.atomic.AtomicReference;

import edu.unc.robotics.prrts.tree.Link;
import edu.unc.robotics.prrts.tree.Node;

import java.util.logging.Level;
import java.util.logging.Logger;

/** Utility class for path operations, to make Worker shorter. */
public class Operations {
    private static final Logger _log = Logger.getLogger(Operations.class.getName());

    /**
     * If link is a shorter solution than bestPath, set bestPath to link.
     */
    public static void updateBestPath(AtomicReference<Link> bestPath, Link link) {
        if (!link.get_node().is_inGoal()) {
            return;
        }

        final double distToGoal = link.get_pathDist();

        Link currentBestPath;
        do {
            currentBestPath = bestPath.get();
            if (currentBestPath != null) {
                double bestDist = currentBestPath.get_pathDist();
                if (distToGoal >= bestDist) {
                    return;
                }
            }
        } while (!bestPath.compareAndSet(currentBestPath, link));
    }

    /**
     * Move all the children of oldParent to newParent.
     */
    public static void updateChildren(
            AtomicReference<Link> bestPath,
            Link newParent,
            Link oldParent,
            double radius,
            int threadCount) {
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
                    newParent = oldParent.get_node().get_link().get();
                    continue;
                }

                return;
            }

            if (oldChild.isExpired()) {
                if (threadCount == 1) {
                    // this shouldn't happen but it's not that big a deal
                    _log.log(Level.WARNING, "found expired child with one thread");
                }
                continue;
            }

            Node node = oldChild.get_node();

            // if (node.get_link().get().get_parent().get_node() != oldParent.get_node()) {
            if (node.get_link().get().get_parent_node() != oldParent.get_node()) {
                continue;
            }

            Link newChild = node.setLink(oldChild, oldChild.get_linkDist(), newParent);

            if (newChild != null) {
                updateChildren(bestPath, newChild, oldChild, radius, threadCount);
                updateBestPath(bestPath, newChild);
            } else {
                if (threadCount == 1) {
                    // this shouldn't happen but it's not that big a deal
                    _log.log(Level.WARNING, "found update conflict with one thread");
                }
                if (node.get_link().get() == oldChild) {
                    _log.log(Level.WARNING, "weird child situation");
                }
            }
        }
    }

    public static void rewire(
            AtomicReference<Link> _bestPath,
            RobotModel _robotModel,
            Link oldLink,
            double linkDist,
            Node newParent,
            double radius,
            int _threadCount) {
        if (oldLink.get_parent_node() == null) {
            _log.log(Level.WARNING, "attempted to rewire the root");
            return;
        }
        if (oldLink.get_parent_node() == newParent) {
            _log.log(Level.WARNING, "attempted to rewire to current parent");
            return;
        }

        Node node = oldLink.get_node();

        Link parentLink = newParent.get_link().get();

        double pathDist = parentLink.get_pathDist() + linkDist;

        // check if rewiring would create a shorter path
        if (pathDist >= oldLink.get_pathDist()) {
            return;
        }

        // check if rewiring is possible
        if (!_robotModel.link(oldLink.get_node().get_config(), newParent.get_config())) {
            return;
        }

        // rewire the node. this loop continues to attempt atomic
        // updates until either the update succeeds or the pathDist
        // of the oldLink is found to be better than what we're trying
        // to put in
        do {

            Link newLink = node.setLink(oldLink, linkDist, parentLink);

            if (newLink != null) {
                Operations.updateChildren(_bestPath, newLink, oldLink, radius, _threadCount);
                Operations.updateBestPath(_bestPath, newLink);

                if (parentLink.isExpired()) {
                    Operations.updateChildren(_bestPath,
                            parentLink.get_node().get_link().get(), parentLink, radius, _threadCount);
                }

                // Setting newLink expires oldLink but doesn not remove
                // it from its parent. Here we do a little cleanup.
                // We do it after the expired parent check since the parent
                // will likely have already cleaned this up, and this call
                // will be O(1) instead of O(n)
                if (!oldLink.get_parent().removeChild(oldLink)) {
                    if (_threadCount == 1) {
                        _log.log(Level.WARNING, "concurrent update running with 1 thread");
                    }
                }
                return;
            }
            if (_threadCount == 1) {
                _log.log(Level.WARNING, "concurrent update running with 1 thread");
            }
            
            Link updatedOldLink = node.get_link().get();

            if (updatedOldLink == oldLink) {
                _log.log(Level.WARNING, "update isn't different");
            }

            oldLink = updatedOldLink;

        } while (pathDist < oldLink.get_pathDist());
    }

}
