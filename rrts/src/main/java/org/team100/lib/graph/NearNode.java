package org.team100.lib.graph;

/**
 * Near nodes can be sorted by their goodness, so we try
 * to link to the best, i.e. shortest-total-distance, first.
 */
public class NearNode implements Comparable<NearNode> {
    public final Node node;
    public final double linkDist;
    private final double _pathDist;

    public NearNode(Node node, double linkDist) {
        this.node = node;
        this.linkDist = linkDist;
        if (node.getIncoming() == null) {
            this._pathDist = linkDist;
        } else {
            this._pathDist = node.getIncoming().get_pathDist() + linkDist;
        }
    }

    @Override
    public int compareTo(NearNode o) {
        return Double.compare(_pathDist, o._pathDist);
    }
}
