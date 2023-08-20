package edu.unc.robotics.prrts.kdtree;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import edu.unc.robotics.prrts.tree.Point;

public class Util {

    public static <V extends Point> List<V> values(KDNode<V> root) {
        List<V> list = new ArrayList<V>();
        buildList(list, root);
        return list;
    }

    static <V extends Point> void buildList(List<V> list, KDNode<V> node) {
        if (node == null)
            return;
        list.add(node.getValue());
        buildList(list, node.getA());
        buildList(list, node.getB());
    }

    public static <V extends Point> void insert(KDModel model, KDNode<V> root, V value) {
        double[] min = model.getMin();
        double[] max = model.getMax();

        KDNode<V> newNode = new KDNode<V>(value);
        KDNode<V> n = root;
        int depth = 0;

        for (;; ++depth) {
            int axis = depth % model.dimensions();
            double mp = (min[axis] + max[axis]) / 2;
            double v = value.get_config()[axis];

            if (v < mp) {
                // a-side
                if (n.getA() == null) {
                    n.setA(newNode);
                    break;
                }
                max[axis] = mp;
                n = n.getA();
            } else {
                // b-side
                if (n.getB() == null) {
                    n.setB(newNode);
                    break;
                }
                min[axis] = mp;
                n = n.getB();
            }
        }
    }

    public static <V extends Point> void near(KDModel model, KDNode<V> root, double[] target, double radius,
            BiConsumer<V, Double> consumer) {
        double[] min = model.getMin();
        double[] max = model.getMax();
        Util.near(model, min, max, consumer, root, target, radius, 0);
    }

    public static <V extends Point> void near(KDModel model, double[] min, double[] max, BiConsumer<V, Double> consumer, KDNode<V> n,
            double[] target, double radius, int depth) {
        final double d = model.dist(n.getConfig(), target);
        if (d < radius) {
            consumer.accept(n.getValue(), d);
        }
        final int axis = depth % model.dimensions();
        final double mp = (min[axis] + max[axis]) / 2;
        final double dm = Math.abs(mp - target[axis]);

        KDNode<V> a = n.getA();

        if (a != null && (target[axis] < mp || dm < radius)) {
            // in or near a-side
            double tmp = max[axis];
            max[axis] = mp;
            near(model, min, max, consumer, a, target, radius, depth + 1);
            max[axis] = tmp;
        }

        KDNode<V> b = n.getB();

        if (b != null && (mp <= target[axis] || dm < radius)) {
            // in or near b-side
            double tmp = min[axis];
            min[axis] = mp;
            near(model, min, max, consumer, b, target, radius, depth + 1);
            min[axis] = tmp;
        }
    }

    public static <V extends Point> KDNearNode<V> nearest(KDModel model, KDNode<V> root, double[] target) {
        double[] min = model.getMin();
        double[] max = model.getMax();
        return Util.nearest(new KDNearNode<V>(Double.MAX_VALUE, null), model, root, min, max, target, 0);
    }

    public static <V extends Point> KDNearNode<V> nearest(KDNearNode<V> best, KDModel model, KDNode<V> n,
            double[] min, double[] max,
            double[] target, int depth) {
        final int axis = depth % model.dimensions();
        final double d = model.dist(n.getConfig(), target);

        if (d < best._dist) {
            best = new KDNearNode<V>(d, n.getValue());
        }

        final double mp = (min[axis] + max[axis]) / 2;

        if (target[axis] < mp) {
            // a-side
            KDNode<V> a = n.getA();
            if (a != null) {
                double tmp = max[axis];
                max[axis] = mp;
                best = nearest(best, model, a, min, max, target, depth + 1);
                max[axis] = tmp;
            }

            KDNode<V> b = n.getB();
            if (b != null) {
                double tmp = Math.abs(mp - target[axis]);
                if (tmp < best._dist) {
                    tmp = min[axis];
                    min[axis] = mp;
                    best = nearest(best, model, b, min, max, target, depth + 1);
                    min[axis] = tmp;
                }
            }
        } else {
            // b-side
            KDNode<V> b = n.getB();
            if (b != null) {
                double tmp = min[axis];
                min[axis] = mp;
                best = nearest(best, model, b, min, max, target, depth + 1);
                min[axis] = tmp;
            }

            KDNode<V> a = n.getA();
            if (a != null) {
                double tmp = Math.abs(mp - target[axis]);
                if (tmp < best._dist) {
                    tmp = max[axis];
                    max[axis] = mp;
                    best = nearest(best, model, a, min, max, target, depth + 1);
                    max[axis] = tmp;
                }
            }
        }
        return best;
    }

}
