package org.team100.lib.space;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import org.team100.lib.index.KDModel;


public class Path implements Comparable<Path> {

    /** The total length of the computed path     */
    private final double distance;

    /** The states along the path.   */
    private final List<double[]> states;

    public Path(double distance, List<double[]> states) {
        this.distance = distance;
        this.states = states;
    }

    /**
     * Paths are comparable by their distances.
     *
     * @param that the path to compare to
     * @return -1 if this path is shorter, 0 is equal in length, 1 if longer
     * than the argument path.
     */
    @Override
    public int compareTo(Path that) {
        return Double.compare(distance, that.distance);
    }

    public void interpolate(double[] outConfig, double offset, KDModel kdModel) {
        Iterator<double[]> pathIter = states.iterator();
        double[] from = pathIter.next();
        double[] dest = pathIter.next();
        double distToFrom = 0;
        double distToDest = kdModel.dist(from, dest);

        while (pathIter.hasNext() && distToDest < offset) {
            from = dest;
            distToFrom = distToDest;
            dest = pathIter.next();
            distToDest += kdModel.dist(from, dest);
        }
        double distBetween = distToDest - distToFrom;
        offset -= distToFrom;
        offset /= distBetween;

        double s = offset;
        double p = 1.0 - s;

        for (int i=0 ; i<kdModel.dimensions() ; ++i) {
            outConfig[i] = from[i] * p + dest[i] * s;
        }
    }

    public static boolean isBetter(Path a, Path b) {
        if (a == null) {
            return false;
        }
        if (b == null) {
            return true;
        }
        return a.distance < b.distance;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Path)) return false;

        Path path = (Path) o;

        if (Double.compare(path.distance, distance) != 0) return false;
        if (states.size() != path.states.size()) return false;
        Iterator<double[]> i1 = states.iterator();
        Iterator<double[]> i2 = path.states.iterator();
        while (i1.hasNext()) {
            if (!Arrays.equals(i1.next(), i2.next())) {
                 return false;
            }
        }
        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = distance != +0.0d ? Double.doubleToLongBits(distance) : 0L;
        result = (int) (temp ^ (temp >>> 32));
        for (double[] config : states) {
            result = 31 * result + Arrays.hashCode(config);
        }
        return result;
    }

    public double getDistance() {
        return distance;
    }

    public List<double[]> getStates() {
        return states;
    }

    @Override
    public String toString() {
        return "Path [_dist=" + String.format("%8.5f", distance) + ", _configs=" + states + "]";
    }
}
