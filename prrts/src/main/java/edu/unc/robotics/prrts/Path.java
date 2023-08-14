package edu.unc.robotics.prrts;

import java.util.Iterator;
import java.util.List;

import edu.unc.robotics.prrts.kdtree.KDModel;

/**
 * Represent a computed path, the result of an PRRTStar run.
 *
 * @author jeffi
 */
public class Path<T extends State> implements Comparable<Path<T>> {

    /**
     * The distance of the computed path
     */
    public double dist;

    /**
     * The configurations of the path.
     */
    public List<T> configs;

    public Path(double dist, List<T> configs) {
        this.dist = dist;
        this.configs = configs;
    }

    /**
     * Paths are comparable by their distances.
     *
     * @param that the path to compare to
     * @return -1 if this path is shorter, 0 is equal in length, 1 if longer
     * than the argument path.
     */
    @Override
    public int compareTo(Path<T> that) {
        return Double.compare(this.dist, that.dist);
    }

    public void interpolate(T outConfig, double offset, KDModel<T> kdModel) {
        Iterator<T> pathIter = configs.iterator();
        T from = pathIter.next();
        T dest = pathIter.next();
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

        // TODO: remove this use of get and set
        for (int i=0 ; i<kdModel.dimensions() ; ++i) {
            outConfig.set(i, from.get(i) * p + dest.get(i) * s);
        }
    }

    public static <U extends State> boolean isBetter(Path<U> a, Path<U> b) {
        if (a == null) {
            return false;
        }
        if (b == null) {
            return true;
        }
        return a.dist < b.dist;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Path<?>)) return false;

        Path<?> path = (Path<?>) o;

        if (Double.compare(path.dist, dist) != 0) return false;
        if (configs.size() != path.configs.size()) return false;
        Iterator<T> i1 = configs.iterator();
        Iterator<?> i2 = path.configs.iterator();
        while (i1.hasNext()) {
            if (!i1.next().equals(i2.next())) return false;
        }
        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = dist != +0.0d ? Double.doubleToLongBits(dist) : 0L;
        result = (int) (temp ^ (temp >>> 32));
        for (T config : configs) {
            result = 31 * result + config.hashCode();
        }
        return result;
    }
}
