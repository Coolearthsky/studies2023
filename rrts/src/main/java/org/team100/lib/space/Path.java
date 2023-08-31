package org.team100.lib.space;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;


public class Path implements Comparable<Path> {

    /** The total length of the computed path     */
    private final double distance;

    /** The states along the path.  It's two lists so i can see where the join is. */
    private final List<double[]> states_A;
    private final List<double[]> states_B;

    public Path(double distance, List<double[]> states_A, List<double[]> states_B) {
        this.distance = distance;
        this.states_A = states_A;
        this.states_B = states_B;
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

    // public void interpolate(double[] outConfig, double offset, KDModel kdModel) {
    //     Iterator<double[]> pathIter = states.iterator();
    //     double[] from = pathIter.next();
    //     double[] dest = pathIter.next();
    //     double distToFrom = 0;
    //     double distToDest = kdModel.dist(from, dest);

    //     while (pathIter.hasNext() && distToDest < offset) {
    //         from = dest;
    //         distToFrom = distToDest;
    //         dest = pathIter.next();
    //         distToDest += kdModel.dist(from, dest);
    //     }
    //     double distBetween = distToDest - distToFrom;
    //     offset -= distToFrom;
    //     offset /= distBetween;

    //     double s = offset;
    //     double p = 1.0 - s;

    //     for (int i=0 ; i<kdModel.dimensions() ; ++i) {
    //         outConfig[i] = from[i] * p + dest[i] * s;
    //     }
    // }

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
        return statesEqual(states_A, path.states_A) && statesEqual(states_B, path.states_B);
    }

    private boolean statesEqual(List<double[]> states, List<double[]> otherstates) {
        if (states.size() != otherstates.size()) return false;
        Iterator<double[]> i1 = states.iterator();
        Iterator<double[]> i2 = otherstates.iterator();
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
        for (double[] config : states_A) {
            result = 31 * result + Arrays.hashCode(config);
        }
        for (double[] config : states_B) {
            result = 31 * result + Arrays.hashCode(config);
        }
        return result;
    }

    public double getDistance() {
        return distance;
    }

    public List<double[]> getStatesA() {
        List<double[]> allStates = new ArrayList<double[]>();
        allStates.addAll(states_A);
        return allStates;
    }
    
    public List<double[]> getStatesB() {
        List<double[]> allStates = new ArrayList<double[]>();
        allStates.addAll(states_B);
        return allStates;
    }

    @Override
    public String toString() {
        String result="";
        result += "Path [_dist=" + String.format("%8.5f", distance);
        result += " states_A=[\n";
        for (double[] d : states_A) {
            String[] fmtted = new String[d.length];
            for (int i = 0; i < d.length; ++i) {
                fmtted[i] = String.format("%7.3f", d[i]);
            }
            result += "  [" + String.join(", ", fmtted) + "]\n";
        } 
        result += "]\n";
        result += " states_B=[\n";
        for (double[] d : states_B) {
            String[] fmtted = new String[d.length];
            for (int i = 0; i < d.length; ++i) {
                fmtted[i] = String.format("%7.3f", d[i]);
            }
            result += "  [" + String.join(", ", fmtted) + "]\n";
        } 
        result += "]]";
    return result;
    }
}
