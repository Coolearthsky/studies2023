package edu.unc.robotics.prrts.tree;

public class NearNode implements Comparable<NearNode> {
    public Link link;
    public double linkDist;
    public double pathDist;

    @Override
    public int compareTo(NearNode o) {
        return Double.compare(this.pathDist, o.pathDist);
    }
}
