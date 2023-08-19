package edu.unc.robotics.prrts.kdtree;

import java.util.function.BiConsumer;

class Traversal<V> implements KDTraversal<V> {
    private final KDModel _model;
    private final KDNode<V> _root;
    private final int _dimensions;
    final double[] _min;
    final double[] _max;
    // updated by nearest()
    private double _dist;
    private V _nearest;

    public Traversal(KDModel model, KDNode<V> root, int dimensions) {
        _model = model;
        _root = root;
        _dimensions = dimensions;
        _min = new double[_dimensions];
        _max = new double[_dimensions];
    }

    @Override
    public double distToLastNearest() {
        return _dist;
    }

    @Override
    public void insert(double[] config, V value) {
        double[] min = _min;
        double[] max = _max;

        _model.getBounds(min, max);

        KDNode<V> newNode = new KDNode<V>(config, value);
        KDNode<V> n = _root;
        int depth = 0;

        for (;; ++depth) {
            int axis = depth % _dimensions;
            double mp = (min[axis] + max[axis]) / 2;
            double v = config[axis];

            if (v < mp) {
                // a-side
                if (n.getA() == null) {
                    if (n.setA(null, newNode)) {
                        break;
                    }
                }
                _max[axis] = mp;
                n = n.getA();
            } else {
                // b-side
                if (n.getB() == null) {
                    if (n.setB(null, newNode)) {
                        break;
                    }
                }
                _min[axis] = mp;
                n = n.getB();
            }
        }
    }

    public V nearest(double[] target) {
        _dist = Double.MAX_VALUE;
        _model.getBounds(_min, _max);
        nearest(_root, target, 0);
        return _nearest;
    }

    /**
     * returns when it's done digging; it leaves the answers in _nearest and _dist.
     * yuck.
     * TODO: recast as immutable
     */
    private void nearest(KDNode<V> n, double[] target, int depth) {
        final int axis = depth % _dimensions;
        final double d = _model.dist(n.getConfig(), target);

        if (d < _dist) {
            _dist = d;
            _nearest = n.getValue();
        }

        final double mp = (_min[axis] + _max[axis]) / 2;

        if (target[axis] < mp) {
            // a-side
            KDNode<V> a = n.getA();
            if (a != null) {
                double tmp = _max[axis];
                _max[axis] = mp;
                nearest(a, target, depth + 1);
                _max[axis] = tmp;
            }

            KDNode<V> b = n.getB();
            if (b != null) {
                double tmp = Math.abs(mp - target[axis]);
                if (tmp < _dist) {
                    tmp = _min[axis];
                    _min[axis] = mp;
                    nearest(b, target, depth + 1);
                    _min[axis] = tmp;
                }
            }
        } else {
            // b-side
            KDNode<V> b = n.getB();
            if (b != null) {
                double tmp = _min[axis];
                _min[axis] = mp;
                nearest(b, target, depth + 1);
                _min[axis] = tmp;
            }

            KDNode<V> a = n.getA();
            if (a != null) {
                double tmp = Math.abs(mp - target[axis]);
                if (tmp < _dist) {
                    tmp = _max[axis];
                    _max[axis] = mp;
                    nearest(a, target, depth + 1);
                    _max[axis] = tmp;
                }
            }
        }
    }

    @Override
    public void near(double[] target, double radius, BiConsumer<V, Double> consumer) {
        _model.getBounds(_min, _max);
        near(consumer, _root, target, radius, 0);
    }

    private void near(BiConsumer<V, Double> consumer, KDNode<V> n, double[] target, double radius, int depth) {
        final double d = _model.dist(n.getConfig(), target);
        if (d < radius) {
            consumer.accept(n.getValue(), d);
        }
        final int axis = depth % _dimensions;
        final double mp = (_min[axis] + _max[axis]) / 2;
        final double dm = Math.abs(mp - target[axis]);

        KDNode<V> a = n.getA();

        if (a != null && (target[axis] < mp || dm < radius)) {
            // in or near a-side
            double tmp = _max[axis];
            _max[axis] = mp;
            near(consumer, a, target, radius, depth + 1);
            _max[axis] = tmp;
        }

        KDNode<V> b = n.getB();

        if (b != null && (mp <= target[axis] || dm < radius)) {
            // in or near b-side
            double tmp = _min[axis];
            _min[axis] = mp;
            near(consumer, b, target, radius, depth + 1);
            _min[axis] = tmp;
        }
    }
}