package edu.unc.robotics.prrts;

import edu.unc.robotics.prrts.kdtree.KDModel;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

/**
 * Represent a computed path, the result of an PRRTStar run.
 *
 * @author jeffi
 */
public class Path implements Comparable<Path> {

    /**
     * The distance of the computed path
     */
    private final double _dist;

    /**
     * The configurations of the path.
     */
    private final List<double[]> _configs;

    public Path(double dist, List<double[]> configs) {
        _dist = dist;
        _configs = configs;
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
        return Double.compare(_dist, that._dist);
    }

    public void interpolate(double[] outConfig, double offset, KDModel kdModel) {
        Iterator<double[]> pathIter = _configs.iterator();
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
        return a._dist < b._dist;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Path)) return false;

        Path path = (Path) o;

        if (Double.compare(path._dist, _dist) != 0) return false;
        if (_configs.size() != path._configs.size()) return false;
        Iterator<double[]> i1 = _configs.iterator();
        Iterator<double[]> i2 = path._configs.iterator();
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
        temp = _dist != +0.0d ? Double.doubleToLongBits(_dist) : 0L;
        result = (int) (temp ^ (temp >>> 32));
        for (double[] config : _configs) {
            result = 31 * result + Arrays.hashCode(config);
        }
        return result;
    }

    public double get_dist() {
        return _dist;
    }

    public List<double[]> get_configs() {
        return _configs;
    }

    @Override
    public String toString() {
        return "Path [_dist=" + String.format("%8.5f", _dist) + ", _configs=" + _configs + "]";
    }
}
