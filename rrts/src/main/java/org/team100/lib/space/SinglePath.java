package org.team100.lib.space;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * paths from both trees joined into a single list.
 */
public class SinglePath<States extends Num> {
    public static class Link<States extends Num> {
        public final Matrix<States, N1> x_i;
        public final Matrix<States, N1> x_g;
        public final double cost;

        public Link(Matrix<States, N1> x_i, Matrix<States, N1> x_g, double cost) {
            this.x_i = x_i;
            this.x_g = x_g;
            this.cost = cost;
        }
    }

    /** The total length of the computed path */
    private final double distance;

    /** The states along the path. */
    private final List<Matrix<States, N1>> states;
    private final List<Link<States>> links;

    public SinglePath(double distance, List<Matrix<States, N1>> states, List<Link<States>> links) {
        this.distance = distance;
        this.states = states;
        this.links = links;

        // invariant: x_g of one link is x_i of the next.
        for (int i = 0; i < links.size() - 1; ++i) {
            Link<States> link1 = links.get(i);
            Link<States> link2 = links.get(i + 1);
            if (!link1.x_g.isEqual(link2.x_i, 0.001)) {
                System.out.printf("%d %s %s\n", i, link1.x_g, link2.x_i);
                throw new IllegalArgumentException();
            }
        }

        // these are temporary
        if (states.size() != links.size() + 1)
            throw new IllegalArgumentException();
        for (int i = 0; i < links.size(); ++i) {
            Matrix<States, N1> initial = states.get(i);
            Matrix<States, N1> goal = states.get(i + 1);
            Link<States> link = links.get(i);
            if (!initial.isEqual(link.x_i, 0.001))
                throw new IllegalArgumentException();
            if (!goal.isEqual(link.x_g, 0.001))
                throw new IllegalArgumentException();
        }
    }

    public double getDistance() {
        return distance;
    }

    public List<Matrix<States, N1>> getStates() {
        List<Matrix<States, N1>> allStates = new ArrayList<>();
        allStates.addAll(states);
        return allStates;
    }

    /** return a copy of the link list */
    public List<Link<States>> getLinks() {
        List<Link<States>> allLinks = new ArrayList<>();
        allLinks.addAll(links);
        return allLinks;
    }

    public Matrix<States, N1> getRoot() {
        return states.get(0);
    }

    public Link<States> getFirstLink() {
        return links.get(0);
    }

    public Link<States> getLastLink() {
        return links.get(links.size() - 1);
    }
}
