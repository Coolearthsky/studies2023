package edu.unc.robotics.prrts.kdtree;

import java.util.concurrent.atomic.AtomicReference;

final class KDNode<V> {
    final double[] config;
    final V value;

    final AtomicReference<KDNode<V>> a = new AtomicReference<>();
    final AtomicReference<KDNode<V>> b = new AtomicReference<>();

    KDNode(double[] c, V v) {
        assert c != null && v != null;
        config = c;
        value = v;
    }

    boolean setA(KDNode<V> old, KDNode<V> n) {
        return a.compareAndSet(old, n);
    }

    boolean setB(KDNode<V> old, KDNode<V> n) {
        return b.compareAndSet(old, n);
    }

    public KDNode<V> getA() {
        return a.get();
    }

    public KDNode<V> getB() {
        return b.get();
    }
}