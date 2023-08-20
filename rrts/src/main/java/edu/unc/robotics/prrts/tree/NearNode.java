package edu.unc.robotics.prrts.tree;

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
