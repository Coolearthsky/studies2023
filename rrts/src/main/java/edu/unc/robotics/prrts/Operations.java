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
        if (!link.get_target().is_inGoal()) {
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
        if (oldLink.get_source() == null) {
            _log.log(Level.WARNING, "attempted to rewire the root");
            return _bestPath;
        }
        if (oldLink.get_source() == newParent) {
            _log.log(Level.WARNING, "attempted to rewire to current parent");
            return _bestPath;
        }

        Link newParentLink = newParent.get_incoming();

        double newPathDist = newParentLink.get_pathDist() + linkDist;

        // if the new path is not better, then return the old path
        if (newPathDist >= oldLink.get_pathDist()) {
            return _bestPath;
        }

        // if the new link is not feasible, return the old path
        if (!_robotModel.link(oldLink.get_target().get_config(), newParent.get_config())) {
            return _bestPath;
        }

        Link newLink = oldLink.get_target().setLink(linkDist, newParentLink);
        return Operations.updateBestPath(_bestPath, newLink);
    }
}
