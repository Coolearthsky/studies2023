package org.team100.lib.space;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * paths from both trees joined into a single list.
 */
public class SinglePath<States extends Num> implements Comparable<SinglePath<States>> {

    /** The total length of the computed path */
    private final double distance;

    /** The states along the path. */
    private final List<Matrix<States, N1>> states;

    public SinglePath(double distance, List<Matrix<States, N1>> states) {
        this.distance = distance;
        this.states = states;
    }

    @Override
    public int compareTo(SinglePath<States> that) {
        return Double.compare(distance, that.distance);
    }

    public static <States extends Num> boolean isBetter(SinglePath<States> a, SinglePath<States> b) {
        if (a == null) {
            return false;
        }
        if (b == null) {
            return true;
        }
        return a.distance < b.distance;
    }

    public double getDistance() {
        return distance;
    }

    public List<Matrix<States, N1>> getStates() {
        List<Matrix<States, N1>> allStates = new ArrayList<>();
        allStates.addAll(states);
        return allStates;
    }

}
