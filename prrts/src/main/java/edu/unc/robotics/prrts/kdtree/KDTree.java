package edu.unc.robotics.prrts.kdtree;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import edu.unc.robotics.prrts.State;

/**
 * KDTree
 *
 * @author jeffi
 */
public class KDTree<T extends State,V> {

    private static final class Node<T, V> {
        final T config;
        final V value;

        final AtomicReference<Node<T,V>> a = new AtomicReference<>();
        final AtomicReference<Node<T,V>> b = new AtomicReference<>();

        Node(T c, V v) {
            assert c != null && v != null;
            config = c;
            value = v;
        }

        boolean setA(Node<T,V> old, Node<T,V> n) {
            return a.compareAndSet(old, n);
        }

        boolean setB(Node<T,V> old, Node<T,V> n) {
            return b.compareAndSet(old, n);
        }

        public Node<T,V> getA() {
            return a.get();
        }

        public Node<T,V> getB() {
            return b.get();
        }
    }

    final KDModel<T>  _model;
    final int _dimensions;
    final Node<T,V> _root;

    public KDTree(KDModel<T> model, T rootConfig, V rootValue) {
        _model = model;
        _dimensions = model.dimensions();
        _root = new Node<>(rootConfig, rootValue);
    }

    public Iterable<V> values() {
        List<V> list = new ArrayList<V>();
        buildList(list, _root);
        return list;
    }

    private void buildList(List<V> list, Node<T,V> node) {
        if (node != null) {
            list.add(node.value);
            buildList(list, node.a.get());
            buildList(list, node.b.get());
        }
    }

    public KDTraversal<T,V> newTraversal() {
        return new Traversal();
    }

    class Traversal implements KDTraversal<T,V> {
        private final int _dimensions = KDTree.this._dimensions;
        private final T _min = _model.getMin();
        private final T _max = _model.getMax();

        int _statConcurrentInserts;

        public double _dist;
        public V _nearest;

        @Override
        public double distToLastNearest() {
            return _dist;
        }

        @Override
        public void insert(T config, V value) {
            T min = _min;
            T max = _max;

            _model.getBounds(min, max);

            Node<T,V> newNode = new Node<>(config, value);
            Node<T,V> n = _root;
            int depth = 0;

            for (;; ++depth) {
                int axis = depth % _dimensions;
                double mp = (min.get(axis) + max.get(axis)) / 2;
                double v = config.get(axis);

                if (v < mp) {
                    // a-side
                    if (n.getA() == null) {
                        if (n.setA(null, newNode)) {
                            break;
                        }
                        _statConcurrentInserts++;
                    }
                    _max.set(axis, mp);
                    n = n.getA();
                } else {
                    // b-side
                    if (n.getB() == null) {
                        if (n.setB(null, newNode)) {
                            break;
                        }
                        _statConcurrentInserts++;
                    }
                    _min.set(axis, mp);
                    n = n.getB();
                }
            }
        }

        public V nearest(T target) {
            _dist = Double.MAX_VALUE;
            _model.getBounds(_min, _max);
            nearest(_root, target, 0);
            return _nearest;
        }

        private void nearest(Node<T,V> n, T target, int depth) {
            final int axis = depth % _dimensions;
            final double d = _model.dist(n.config, target);

            if (d < _dist) {
                _dist = d;
                _nearest = n.value;
            }

            final double mp = (_min.get(axis) + _max.get(axis)) / 2;

            if (target.get(axis) < mp) {
                // a-side
                Node<T, V> a = n.getA();
                if (a != null) {
                    double tmp = _max.get(axis);
                    _max.set(axis, mp);
                    nearest(a, target, depth + 1);
                    _max.set(axis, tmp);
                }

                Node<T, V> b = n.getB();
                if (b != null) {
                    double tmp = Math.abs(mp - target.get(axis));
                    if (tmp < _dist) {
                        tmp = _min.get(axis);
                        _min.set(axis, mp);
                        nearest(b, target, depth + 1);
                        _min.set(axis, tmp);
                    }
                }
            } else {
                // b-side
                Node<T, V> b = n.getB();
                if (b != null) {
                    double tmp = _min.get(axis);
                    _min.set(axis, mp);
                    nearest(b, target, depth + 1);
                    _min.set(axis, tmp);
                }

                Node<T, V> a = n.getA();
                if (a != null) {
                    double tmp = Math.abs(mp - target.get(axis));
                    if (tmp < _dist) {
                        tmp = _max.get(axis);
                        _max.set(axis, mp);
                        nearest(a, target, depth + 1);
                        _max.set(axis, tmp);
                    }
                }
            }
        }

        @Override
        public int near(T target, double radius, KDNearCallback<T, V> callback) {
            _model.getBounds(_min, _max);
            return near(_root, target, radius, callback, 0, 0);
        }

        private int near(Node<T, V> n, T target, double radius, KDNearCallback<T, V> callback, int count, int depth) {
            final double d = _model.dist(n.config, target);
            if (d < radius) {
                callback.kdNear(target, count++, n.config, n.value, d);
            }
            final int axis = depth % _dimensions;
            final double mp = (_min.get(axis) + _max.get(axis)) / 2;
            final double dm = Math.abs(mp - target.get(axis));

            Node<T, V> a = n.getA();

            if (a != null && (target.get(axis) < mp || dm < radius)) {
                // in or near a-side
                double tmp = _max.get(axis);
                _max.set(axis, mp);
                count = near(a, target, radius, callback, count, depth + 1);
                _max.set(axis, tmp);
            }

            Node<T, V> b = n.getB();

            if (b != null && (mp <= target.get(axis) || dm < radius)) {
                // in or near b-side
                double tmp = _min.get(axis);
                _min.set(axis, mp);
                count = near(b, target, radius, callback, count, depth + 1);
                _min.set(axis, tmp);
            }

            return count;
        }
    }
}
