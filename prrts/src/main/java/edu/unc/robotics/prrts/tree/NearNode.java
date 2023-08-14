package edu.unc.robotics.prrts.tree;

import edu.unc.robotics.prrts.State;

public class NearNode<T extends State> implements Comparable<NearNode<T>> {
    public Link<T> link;
    public double linkDist;
    public double pathDist;

    @Override
    public int compareTo(NearNode<T> o) {
        return Double.compare(this.pathDist, o.pathDist);
    }
}
