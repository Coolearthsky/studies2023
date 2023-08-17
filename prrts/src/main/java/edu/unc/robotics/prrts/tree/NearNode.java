package edu.unc.robotics.prrts.tree;

public class NearNode implements Comparable<NearNode> {
    public final Link link;
    public final double linkDist;
    public final double _pathDist;

    public NearNode(Link link, double linkDist) {
        this.link = link;
        this.linkDist = linkDist;
        this._pathDist = link.get_pathDist() + linkDist;
    }

    @Override
    public int compareTo(NearNode o) {
        return Double.compare(_pathDist, o._pathDist);
    }
}
