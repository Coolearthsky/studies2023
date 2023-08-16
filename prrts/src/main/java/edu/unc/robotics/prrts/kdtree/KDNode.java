package edu.unc.robotics.prrts.kdtree;

import java.util.concurrent.atomic.AtomicReference;

final class KDNode<V> {
    private final double[] config;
    private final V value;
    private final AtomicReference<KDNode<V>> a;
    private final AtomicReference<KDNode<V>> b;

    KDNode(double[] c, V v) {
        assert c != null && v != null;
        config = c;
        value = v;
        a = new AtomicReference<>();
        b = new AtomicReference<>();
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

    public double[] getConfig() {
        return config;
    }

    public V getValue() {
        return value;
    }
}