package edu.unc.robotics.prrts.tree;

public class NearNode implements Comparable<NearNode> {
    public Link link;
    public double linkDist;
    public double _pathDist;

    @Override
    public int compareTo(NearNode o) {
        return Double.compare(_pathDist, o._pathDist);
    }
}
